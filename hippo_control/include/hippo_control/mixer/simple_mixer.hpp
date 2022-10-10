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

  std::array<double, kChannels> Mix(
      const std::array<double, kChannels> &_actuator_controls);

 private:
  Mapping mappings_[kChannels];
  Output outputs_[kChannels];
  double ApplyInput(const std::array<double, kChannels> &_actuator_controls);
  void ScaleOutputs(double _scale);
  void ResetOutputs();
};
}  // namespace mixer
}  // namespace hippo_control
