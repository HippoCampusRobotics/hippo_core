    #include "hippo_control/mixer/simple_mixer.hpp"

#include <math.h>
#include <iostream>

namespace hippo_control {
namespace mixer {

SimpleMixer::SimpleMixer(){};

void SimpleMixer::SetMapping(int _index, const Mapping &_mapping) {
  if ((_index >= kOutputChannels)) {
    return;
  }
  mappings_[_index] = _mapping;
}

void SimpleMixer::ResetOutputs() {
  for (Output &output : outputs_) {
    output.total = 0.0;
    output.channels.fill(0.0);
  }
}

double SimpleMixer::ApplyInput(
    const std::array<double, InputChannels::kCount> &_actuator_controls) {
  ResetOutputs();
  // scaling factor to scale the maximum output to 1.0, if any output is > 1.0
  double scaling = 1.0;
  for (int i_out = 0; i_out < kOutputChannels; ++i_out) {
    for (int i_in = 0; i_in < InputChannels::kCount; ++i_in) {
      double tmp =
          _actuator_controls[i_in] * mappings_[i_out].input_scalings[i_in];
      outputs_[i_out].total += tmp;
      outputs_[i_out].channels[i_in] += tmp;
    }
    double thrust = abs(outputs_[i_out].total);
    double output = ThrustToRevsPerSec(thrust);
    output /= max_rotations_per_second_;
    output *= mappings_[i_out].output_scaling;
    if (outputs_[i_out].total < 0) {
      outputs_[i_out].total = -1.0 * output;
    } else {
      outputs_[i_out].total = output;
    }
    scaling = std::max(output, scaling);
  }
  return scaling;
}

double SimpleMixer::ThrustToRevsPerSec(double _thrust) {
  if (_thrust < zero_throttle_threshold_) {
    return 0.0;
  }
  if (linear_coefficient_ == 0.0) {
    if (quadratic_coefficient_ == 0.0) {
      // it does not make sense to have F(n) = const, so return 0.0
      return 0.0;
    }
    // F(n) = an² + c
    return sqrt((_thrust - constant_coefficient_) / quadratic_coefficient_);
  }

  if (quadratic_coefficient_ == 0.0) {
    // F(n) = bn + c
    return (_thrust - constant_coefficient_) / linear_coefficient_;
  }
  // full quadratic polynomial
  return (-1.0 * linear_coefficient_ +
          sqrt(4.0 * quadratic_coefficient_ * _thrust +
               linear_coefficient_ * linear_coefficient_ -
               4.0 * quadratic_coefficient_ * constant_coefficient_)) /
         (2.0 * quadratic_coefficient_);
}

void SimpleMixer::ScaleOutputs(double _scale) {
  for (Output &output : outputs_) {
    output.total /= _scale;
  }
}

std::array<double, kOutputChannels> SimpleMixer::Mix(
    const std::array<double, InputChannels::kCount> &_actuator_controls) {
  double scale = ApplyInput(_actuator_controls);
  if (scale > 1.0) {
    ScaleOutputs(scale);
  }
  std::array<double, kOutputChannels> out;
  for (int i = 0; i < kOutputChannels; ++i) {
    out[i] = outputs_[i].total;
  }
  return out;
}

}  // namespace mixer
}  // namespace hippo_control
