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
  std::array<double, InputChannels::kCount> limits{};
  std::array<double, InputChannels::kCount> scalings{};
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
  void SetLinearCoefficient(double _v) { linear_coefficient_ = _v; }
  inline double LinearCoefficient() const { return linear_coefficient_; }
  void SetQuadraticCoefficient(double _v) { quadratic_coefficient_ = _v; }
  inline double QuadraticCoefficient() const { return quadratic_coefficient_; }
  void SetMaxRotationsPerSecond(double _v) { max_rotations_per_second_ = _v; }
  double MaxRotationsPerSecond() const { return max_rotations_per_second_; }

  std::array<double, kOutputChannels> Mix(
      const std::array<double, InputChannels::kCount> &_actuator_controls);

 private:
  /// @brief per motor mappings of torque/thrust
  Mapping mappings_[kOutputChannels];
  Output outputs_[kOutputChannels];
  double linear_coefficient_;
  double quadratic_coefficient_;
  double max_rotations_per_second_{1.0};
  double ApplyInput(
      const std::array<double, InputChannels::kCount> &_actuator_controls);
  void ScaleOutputs(double _scale);
  void ResetOutputs();
};
}  // namespace mixer
}  // namespace hippo_control
