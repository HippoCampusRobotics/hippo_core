#include "bridge.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "qualisys_bridge/qualisys/RTProtocol.h"

RCLCPP_COMPONENTS_REGISTER_NODE(qualisys_bridge::Bridge)

static constexpr double kMaxJerk = 50.0;
static constexpr double kFrameInterval = 0.01;

namespace qualisys_bridge {
Bridge::Bridge(rclcpp::NodeOptions const &_options)
    : Node("apriltag_simple", _options), ekf_() {
  process_noise_.topLeftCorner<6, 6>() = 1.0 / 6.0 * Ekf::Matrix6d::Identity() *
                                         kFrameInterval * kFrameInterval *
                                         kFrameInterval * kMaxJerk;
  process_noise_.block<6, 6>(6, 6) = 0.5 * Ekf::Matrix6d::Identity() *
                                     kFrameInterval * kFrameInterval * kMaxJerk;
  process_noise_.bottomRightCorner<3, 3>() =
      Eigen::Matrix3d::Identity() * kFrameInterval * kMaxJerk;

  measurement_noise_ = Ekf::Matrix6d::Identity() * 1e-3;

  ekf_.Init(process_noise_, measurement_noise_, 100);

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS());

  accel_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "acceleration", rclcpp::SensorDataQoS());

  update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1),
                           std::bind(&Bridge::OnUpdate, this));
}

void Bridge::HandlePacket(CRTPacket *_packet) {
  static double t_prev_frame_qtm = 0.0;
  float x, y, z;
  float R[9];
  double t_packet = _packet->GetTimeStamp() * 1e-6;
  if (t_start_frame_ros_ == 0.0) {
    t_start_frame_ros_ = now().nanoseconds() * 1e-9;
    t_start_frame_qtm_ = _packet->GetTimeStamp() * 1e-6;
  } else if (t_prev_frame_qtm > t_packet) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropping older packet.");
  } else {
    t_prev_frame_qtm = t_packet;
  }
  for (unsigned int i = 0; i < _packet->Get6DOFBodyCount(); ++i) {
    std::string name{rt_protocol_.Get6DOFBodyName(i)};
    if (name != params_.name) {
      continue;
    }
    if (!_packet->Get6DOFBody(i, x, y, z, R)) {
      RCLCPP_WARN(get_logger(), "Failed to retrieve 6DOF data.");
      continue;
    }
    bool matrix_nan{false};
    for (int j = 0; j < 9; ++j) {
      if (std::isnan(R[j])) {
        matrix_nan = true;
      }
    }
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || matrix_nan) {
      ekf_.Reset();
      continue;
    }
    Eigen::Matrix<float, 3, 3, Eigen::ColMajor> R_mat(R);
    Eigen::Quaterniond orientation_measurement{R_mat.cast<double>()};
    Eigen::Vector3d position_measurement{x / 1000.0, y / 1000.0, z / 1000.0};
    position_measurement += Eigen::Vector3d{1.2, 1.8, -1.3};

    double time = t_start_frame_ros_ + (t_packet - t_start_frame_qtm_);

    if (!ekf_.IsReady()) {
      ekf_.SetInitialCondition(time, orientation_measurement,
                               position_measurement);
      continue;
    }
    ekf_.Predict(time);
    ekf_.Update(orientation_measurement, position_measurement);
    PublishOdometry();
    PublishAcceleration();
  }
}

void Bridge::PublishOdometry() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  msg.child_frame_id = "base_link";
  hippo_common::convert::EigenToRos(ekf_.GetPosition(), msg.pose.pose.position);
  hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                    msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(ekf_.GetLinearVelocity(),
                                    msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(
      ekf_.GetOrientation().inverse() * ekf_.GetAngularVelocity(),
      msg.twist.twist.angular);
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance(
      msg.pose.covariance.begin());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_covariance(
      msg.twist.covariance.begin());

  Eigen::Matrix<double, 15, 15> covariance = ekf_.GetStateCovariance();

  pose_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(3, 3);
  pose_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(3, 0);
  pose_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(0, 3);
  pose_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(0, 0);

  twist_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(9, 9);
  twist_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(9, 6);
  twist_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(6, 9);
  twist_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(6, 6);

  odometry_pub_->publish(msg);
}

void Bridge::PublishAcceleration() {
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(ekf_.GetLinearAcceleration(), msg.vector);
  accel_pub_->publish(msg);
}

void Bridge::OnUpdate() {
  if (!rt_protocol_.Connected()) {
    RCLCPP_INFO(get_logger(), "Trying to connect.");
    if (!Connect()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected.");
  }
  if (!data_available_) {
    if (!rt_protocol_.Read6DOFSettings(data_available_)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
  }

  if (!stream_frames_) {
    if (!rt_protocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udp_port_,
                                   NULL, CRTProtocol::cComponent6d)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
    stream_frames_ = true;
    RCLCPP_INFO(get_logger(), "Start streaming.");
  }

  CRTPacket::EPacketType packet_type;
  if (rt_protocol_.Receive(packet_type, true) ==
      CNetwork::ResponseType::success) {
    if (packet_type == CRTPacket::PacketData) {
      CRTPacket *packet = rt_protocol_.GetRTPacket();
      HandlePacket(packet);
    }
  }
}

bool Bridge::Connect() {
  if (!rt_protocol_.Connect(server_address_.c_str(), base_port_, &udp_port_,
                            major_version_, minor_version_, big_endian_)) {
    RCLCPP_ERROR(get_logger(), "Failed to connect!");
    return false;
  }
  return true;
}

}  // namespace qualisys_bridge
