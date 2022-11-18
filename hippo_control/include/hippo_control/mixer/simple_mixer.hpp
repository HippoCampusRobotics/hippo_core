#pragma once
#include <array>
#include <vector>

namespace hippo_control {
namespace mixer {
static constexpr int kChannels = 8;
enum class Channel {
  kRoll,
  kPitch,
  kYaw,
  kThrust,
  kVerticalThrust,
  kLateralThrust,
  kMaxChannel,
};

struct Mapping {
  std::array<double, kChannels> limits{};
  std::array<double, kChannels> scalings{};
};

struct Output {
  double total = 0.0;
  std::array<double, kChannels> channels{};
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

  std::array<double, kChannels> Mix(
      const std::array<double, kChannels> &_actuator_controls);

 private:
  Mapping mappings_[kChannels];
  Output outputs_[kChannels];
  double linear_coefficient_;
  double quadratic_coefficient_;
  double max_rotations_per_second_{1.0};
  double ApplyInput(const std::array<double, kChannels> &_actuator_controls);
  void ScaleOutputs(double _scale);
  void ResetOutputs();
};
}  // namespace mixer
}  // namespace hippo_control
