// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include <fcntl.h>

#include <hippo_control_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/esc_rpms.hpp>
#include <hippo_msgs/msg/esc_voltages.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "afro_esc.h"

using std::placeholders::_1;
using std::placeholders::_2;

static constexpr int kArmingStatePeriodMs = 200;
static constexpr int kVoltagePeriodMs = 500;
static constexpr int kThrottleInputTimeoutMs = 500;
static constexpr int kSendThrottlePeriodMs = 20;

class ESC : public rclcpp::Node {
 public:
  ESC()
      : Node("esc_commander"),
        i2c_addresses_(8),
        timed_out_(false),
        i2c_device_("/dev/i2c-4") {
    Init();
  }
  void Init() {
    RCLCPP_INFO(get_logger(), "Opening i2c device: %s", i2c_device_.c_str());
    i2c_handle_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_handle_ < 0) {
      RCLCPP_FATAL(get_logger(), "Could not open i2c device: '%s'",
                   i2c_device_.c_str());
      exit(1);
    }
    InitEscs();
    paramter_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ESC::onSetParameters, this, _1));
    InitParams();

    esc_voltage_pub_ = this->create_publisher<hippo_msgs::msg::EscVoltages>(
        "esc_voltages", 50);

    battery_voltage_pub_ =
        create_publisher<std_msgs::msg::Float64>("battery_voltage", 50);

    esc_rpm_pub_ =
        this->create_publisher<hippo_msgs::msg::EscRpms>("esc_rpms", 50);

    arming_state_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("arming_state", 10);

    arming_servie_ = create_service<std_srvs::srv::SetBool>(
        "arm", std::bind(&ESC::ServeArming, this, _1, _2));

    control_timeout_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(kThrottleInputTimeoutMs),
        std::bind(&ESC::OnInputTimeout, this));

    // send_throttle_timer_ = rclcpp::create_timer(
    //     this, get_clock(), std::chrono::milliseconds(kSendThrottlePeriodMs),
    //     std::bind(&ESC::OnSendThrottle, this));

    read_battery_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(kVoltagePeriodMs),
        std::bind(&ESC::OnReadBattery, this));

    send_arming_state_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(kArmingStatePeriodMs),
        std::bind(&ESC::OnArmingStateTimer, this));

    rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);
    actuator_controls_sub_ =
        create_subscription<hippo_control_msgs::msg::ActuatorControls>(
            "thruster_command", qos,
            std::bind(&ESC::OnThrusterCommand, this, _1));
  }

  void SetAllThrottle(double _throttle) {
    for (auto &esc : escs_) {
      esc.SetThrottle(_throttle);
    }
  }

  void ServeArming(const std_srvs::srv::SetBool_Request::SharedPtr _request,
                   std_srvs::srv::SetBool_Response::SharedPtr _response) {
    if (_request->data) {
      if (armed_) {
        _response->message = "Already armed.";
        _response->success = false;
        return;
      } else {
        armed_ = _request->data;
        RCLCPP_INFO(get_logger(), "Arming the thrusters.");
        _response->message = "Armed";
        _response->success = false;
        // sending zero throttle initially is required for the ESCs.
        SetThrottleAll(0.0);
        SendThrottle(true);
      }
    } else {
      if (armed_) {
        RCLCPP_INFO(get_logger(), "Disarming the thrusters.");
        armed_ = _request->data;
        // make sure to stop motors immediately
        SetAllThrottle(0.0);
        SendThrottle(true);
        _response->message = "Disarmed";
        _response->success = false;
      } else {
        _response->message = "Already disarmed.";
        _response->success = false;
      }
    }
  }

  void OnArmingStateTimer() {
    std_msgs::msg::Bool msg;
    msg.data = armed_;
    arming_state_pub_->publish(msg);
  }

  void OnInputTimeout() {
    RCLCPP_WARN(get_logger(), "Thruster controls timed out.");
    for (auto &esc : escs_) {
      esc.SetThrottle(0.0);
    }
    SendThrottle(true);
    timed_out_ = true;
    control_timeout_timer_->cancel();
  }

  void OnSendThrottle() { SendThrottle(); }

  void OnReadBattery() {
    auto msg = hippo_msgs::msg::EscVoltages();
    int i = 0;
    double voltage_sum{0.0};
    int n_voltages = 0;
    // either fill data with valid voltages or NaN if communication failed
    for (auto &esc : escs_) {
      if (esc.available()) {
        if (esc.UpdateBatteryAdc() != EscRetCode::kOk) {
          RCLCPP_ERROR(get_logger(),
                       "Failed to read voltage from thruster %d at address %X",
                       esc.index(), esc.address());
          msg.data[i] = std::numeric_limits<double>::quiet_NaN();
        } else {
          msg.data[i] = esc.GetBatteryVoltage();
          voltage_sum += msg.data[i];
          n_voltages++;
        }
      }
      ++i;
    }
    std_msgs::msg::Float64 voltage_msg;
    if (n_voltages > 0) {
      voltage_msg.data = voltage_sum / n_voltages;
    } else {
      voltage_msg.data = std::numeric_limits<double>::quiet_NaN();
    }
    battery_voltage_pub_->publish(voltage_msg);
    esc_voltage_pub_->publish(msg);
  }

  void OnThrusterCommand(
      const hippo_control_msgs::msg::ActuatorControls::SharedPtr msg) {
    // reset/restart the timeout timer since we got a message
    control_timeout_timer_->reset();
    if (timed_out_) {
      timed_out_ = false;
      RCLCPP_INFO(get_logger(),
                  "Received thruster controls. Not timed out anymore");
      SetThrottleAll(0.0);
      SendThrottle(true);
    }
    if (!armed_) {
      SetThrottleAll(0.0);
      return;
    }
    for (int i = 0; i < static_cast<int>(msg->control.size()); i++) {
      if (msg->control[i] != 0 && !escs_[i].InUse()) {
        RCLCPP_WARN(get_logger(),
                    "Setting non-zero thrust for unused ESC at index %d!", i);
      }
      if (!escs_[i].available()) {
        if (msg->control[i] != 0.0) {
          RCLCPP_WARN(
              get_logger(),
              "Setting non-zero thrust for unavailable thruster at index %d",
              i);
        }
        continue;
      }
      escs_[i].SetThrottle(msg->control[i]);
      if (escs_[i].WriteThrottle() != EscRetCode::kOk) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to set motor speed for thruster %d at address %X",
                     escs_[i].index(), escs_[i].address());
      }
    }
  }

  void SetThrottleAll(double _thrust) {
    for (auto &esc : escs_) {
      esc.SetThrottle(_thrust);
    }
  }

  /**
   * @brief Send throttle commands to the ESCs.
   *
   * @param force If true the command is sent even when timed_out_ is true.
   * @return int
   */
  int SendThrottle(bool force = false) {
    int ret = 0;
    hippo_msgs::msg::EscRpms rpm_msg;
    int i = 0;
    if ((timed_out_ || !armed_) && !force) {
      return 0;
    }
    for (auto &esc : escs_) {
      if (esc.available()) {
        if (esc.WriteThrottle() != EscRetCode::kOk) {
          RCLCPP_ERROR(
              get_logger(),
              "Failed to set motor speed for thruster %d at address %X",
              esc.index(), esc.address());
          ret = -1;
        }
        if (esc.UpdateRevolutionCount() != EscRetCode::kOk) {
          RCLCPP_ERROR(get_logger(),
                       "Failed to read rpm from thruster %d at address %X",
                       esc.index(), esc.address());
          rpm_msg.rpms[i] = std::numeric_limits<double>::quiet_NaN();
          rpm_msg.commutations[i] = std::numeric_limits<int64_t>::quiet_NaN();
          rpm_msg.revolutions[i] = std::numeric_limits<double>::quiet_NaN();
        } else {
          rpm_msg.revolutions[i] = esc.GetRevolutionCount();
          rpm_msg.rpms[i] =
              esc.GetRevolutionCount() / kSendThrottlePeriodMs * 1e3;
          rpm_msg.commutations[i] = esc.GetCommutationCount();
        }
      }
      ++i;
    }
    esc_rpm_pub_->publish(rpm_msg);
    return ret;
  }

  void InitParams() {
    RCLCPP_INFO(get_logger(), "Declaring parameters...");
    std::string param_name = "rx_timeout";
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0.0;
    descriptor.floating_point_range[0].to_value = 1.0;
    descriptor.description = "Timeout for motor setpoints.";
    this->declare_parameter<double>(param_name, 0.5, descriptor);

    param_name = "i2c_addresses";
    descriptor.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = 127;
    descriptor.integer_range[0].step = 1;
    std::vector<int64_t> default_value = {41, 42, 43, 44, -1, -1, -1, -1};
    declare_parameter<std::vector<int64_t>>(param_name, default_value,
                                            descriptor);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      RCLCPP_INFO(get_logger(), "Received parameter: %s",
                  param.get_name().c_str());
      if (param.get_name() == "i2c_addresses") {
        bool param_valid;
        param_valid = true;
        RCLCPP_INFO(get_logger(), "Setting i2c_addresses.");
        try {
          if (param.as_integer_array().size() != 8) {
            throw std::length_error("Need exactly 8 i2c addresses.");
          }
        } catch (const rclcpp::ParameterTypeException &e) {
          RCLCPP_ERROR(get_logger(), "Failed to set i2c_addresses: %s",
                       e.what());
          result.successful = false;
          result.reason = "Wrong paramter type for i2c_addresses";
          param_valid = false;
        } catch (const std::length_error &e) {
          RCLCPP_ERROR(get_logger(), "Failed to set i2c_addresses: %s",
                       e.what());
          result.successful = false;
          result.reason =
              "Wrong length for i2c_addresses. Needs exactly 8 values.";
          param_valid = false;
        }
        if (param_valid) {
          i2c_addresses_ = param.as_integer_array();
          UpdateAddresses();
          DetectEscs();
        }
      }
    }
    return result;
  }

  void InitEscs() {
    RCLCPP_INFO(get_logger(), "Init ESCs....");
    int i = 0;
    for (auto const &address : i2c_addresses_) {
      escs_[i].Reset(i2c_handle_, address);
      ++i;
    }
  }

  void UpdateAddresses() {
    RCLCPP_INFO(get_logger(), "Updating ESC addresses...");
    int i = 0;
    for (auto const &address : i2c_addresses_) {
      escs_[i].SetAddress(address);
      escs_[i].SetAvailable(false);
      escs_[i].SetIndex(i);
      escs_[i].SetInUse(address > 0);
      i++;
    }
  }

  void DetectEscs() {
    RCLCPP_INFO(get_logger(), "Detecting ESCs...");
    bool failed = false;

    for (auto &esc : escs_) {
      if (!esc.InUse()) {
        RCLCPP_INFO(get_logger(), "ESC %d unused.", esc.index());
        continue;
      }
      bool ok = false;
      EscRetCode status;
      status = esc.VerifyID(ok);
      if (!((status == EscRetCode::kOk) && ok)) {
        RCLCPP_ERROR(get_logger(),
                     "Could not find ESC at address %X. Error code %d",
                     esc.address(), static_cast<int>(status));
        esc.SetAvailable(false);
        failed = true;
      } else {
        esc.SetAvailable(true);
        RCLCPP_INFO(get_logger(), "Detected ESC at address '%X'",
                    esc.address());
      }
    }
    esc_config_valid_ = !failed;
  }

 private:
  static constexpr int kNumEscs = 8;
  std::vector<int64_t> i2c_addresses_;
  bool esc_config_valid_;
  bool timed_out_;
  bool armed_{false};
  double battery_voltage_{0.0};
  std::array<AfroESC, kNumEscs> escs_;
  std::string i2c_device_;
  int i2c_handle_;
  rclcpp::TimerBase::SharedPtr control_timeout_timer_;
  rclcpp::TimerBase::SharedPtr send_throttle_timer_;
  rclcpp::TimerBase::SharedPtr read_battery_timer_;
  rclcpp::TimerBase::SharedPtr send_arming_state_timer_;
  rclcpp::Subscription<hippo_control_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      paramter_callback_handle_;
  rclcpp::Publisher<hippo_msgs::msg::EscVoltages>::SharedPtr esc_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_pub_;
  rclcpp::Publisher<hippo_msgs::msg::EscRpms>::SharedPtr esc_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_servie_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
