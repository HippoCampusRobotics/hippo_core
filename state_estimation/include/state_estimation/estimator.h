#pragma once
#include <state_estimation/ekf.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hippo_msgs/msg/estimator_innovation.hpp>
#include <hippo_msgs/msg/estimator_sensor_bias.hpp>
#include <hippo_msgs/msg/estimator_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>

struct EstimatorInnovation {
  uint64_t time_us;
  uint64_t time_sample_us;
  double vision_position[3];
  double baro_vertical_position;
  double heading;
};

class Estimator final : public rclcpp::Node {
 public:
  Estimator();
  void InitPublisher();
  //////////////////////////////////////////////////////////////////////////////
  // message callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void OnBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void OnVision(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void BaroUpdate();
  void VisionUpdate();

  void PublishAttitude(const rclcpp::Time &stamp);
  void PublishInnovations(const rclcpp::Time &stamp);
  void PublishPose(const rclcpp::Time &stamp);
  void PublishDelayedPose(const rclcpp::Time &stamp);
  void PublishSensorBias(const rclcpp::Time &stamp);
  void PublishState(const rclcpp::Time &stamp);
  void PublishVelocity(const rclcpp::Time &stamp);

  void ResetImuWatchdog() {
    imu_watchdog_.reset();
    imu_timed_out = false;
  }
  bool IsImuTimedOut() { return imu_timed_out; }
  void OnImuWatchdog();
  void Run();

 private:
  //////////////////////////////////////////////////////////////////////////////
  // Publisher
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::EstimatorInnovation>::SharedPtr
      innovation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      delayed_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      attitude_pub_;
  rclcpp::Publisher<hippo_msgs::msg::EstimatorSensorBias>::SharedPtr
      sensor_bias_pub_;
  rclcpp::Publisher<hippo_msgs::msg::EstimatorState>::SharedPtr state_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // Subscriber
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_sub_;
  rclcpp::TimerBase::SharedPtr imu_watchdog_;
  bool imu_timed_out{false};
  uint64_t imu_time_last_us;
  Ekf ekf_;
  bool baro_updated_{false};
  bool vision_updated_{false};
  BaroSample baro_sample_;
  Eigen::Vector3d gyro_bias_published_;
  Eigen::Vector3d accel_bias_published_;
  VisionSample vision_sample_;
  struct EstimatorParams {
    double baro_atmo_pressure = 101325.0;
    double baro_sealevel_offset = 0.0;
  } params_;
};
