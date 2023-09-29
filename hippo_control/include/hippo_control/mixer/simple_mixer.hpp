#pragma once
#include <array>
#include <vector>

namespace hippo_control {
namespace mixer {
static constexpr int kOutputChannels = 8;
namespace InputChannels {
static constexpr int kTorqueX = 0;
static constexpr int kTorqueY = 1;
static constexpr int kTorqueZ = 2;
static constexpr int kThrustX = 3;
static constexpr int kThrustY = 4;
static constexpr int kThrustZ = 5;
static constexpr int kCount = 6;
}  // namespace InputChannels

struct Mapping {
  std::array<double, InputChannels::kCount> input_limits{};
  std::array<double, InputChannels::kCount> input_scalings{};
  double output_scaling;
};

struct Output {
  double total = 0.0;
  std::array<double, kOutputChannels> channels{};
};

class SimpleMixer {
 public:
  SimpleMixer();
  double test;

  void SetMapping(int _index, const Mapping &_mapping);
  void SetZeroThrustThreshold(double _v) { zero_throttle_threshold_ = _v; }
  inline double ZeroThrustThreshold() const { return zero_throttle_threshold_; }
  void SetConstantCoefficient(double _v) { constant_coefficient_ = _v; }
  inline double ConstantCoefficient() const { return constant_coefficient_; }
  void SetLinearCoefficient(double _v) { linear_coefficient_ = _v; }
  inline double LinearCoefficient() const { return linear_coefficient_; }
  void SetQuadraticCoefficient(double _v) { quadratic_coefficient_ = _v; }
  inline double QuadraticCoefficient() const { return quadratic_coefficient_; }
  void SetMaxRotationsPerSecond(double _v) { max_rotations_per_second_ = _v; }
  double MaxRotationsPerSecond() const { return max_rotations_per_second_; }

  std::array<double, kOutputChannels> Mix(
      const std::array<double, InputChannels::kCount> &_actuator_controls);

 private:
  double ThrustToRevsPerSec(double _thrust);
  /// @brief per motor mappings of torque/thrust commands
  Mapping mappings_[kOutputChannels];
  Output outputs_[kOutputChannels];
  double zero_throttle_threshold_;
  double constant_coefficient_;
  double linear_coefficient_;
  double quadratic_coefficient_;
  // used as a scaler to normalize the motor command
  double max_rotations_per_second_{1.0};
  double ApplyInput(
      const std::array<double, InputChannels::kCount> &_actuator_controls);
  void ScaleOutputs(double _scale);
  void ResetOutputs();
};
}  // namespace mixer
}  // namespace hippo_control
