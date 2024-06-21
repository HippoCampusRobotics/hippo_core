#pragma once

#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace hippo_control {
namespace mixer_bluerov {
static constexpr int kOutputChannels = 8;
static constexpr int kOutputHorizontal = 4;
static constexpr int kOutputVertical = 4;
static constexpr std::array<int, kOutputHorizontal> kOutputIdxsHorizontal = {
    0, 1, 2, 3};
static constexpr std::array<int, kOutputVertical> kOutputIdxsVertical = {4, 5,
                                                                         6, 7};

namespace InputChannels {
static constexpr int kTorqueX = 0;
static constexpr int kTorqueY = 1;
static constexpr int kTorqueZ = 2;
static constexpr int kThrustX = 3;
static constexpr int kThrustY = 4;
static constexpr int kThrustZ = 5;
static constexpr int kCount = 6;
static constexpr int kInputHorizontal = 3;
static constexpr int kInputVertical = 3;
static constexpr std::array<int, kInputHorizontal> kIdxsHorizontal = {2, 3, 4};
static constexpr std::array<int, kInputVertical> kIdxsVertical = {0, 1, 5};
}  // namespace InputChannels

enum ThrustDirection {
  forward,
  backward,
};

struct Mapping {
  std::array<double, InputChannels::kCount> input_limits{};
  double output_scaling;
};

struct Output {
  double total = 0.0;
  std::array<double, kOutputChannels> channels{};
};

struct ThrusterModel {
  double quadratic_coefficient;
  double linear_coefficient;
  double constant_coefficient;
  double minimum;
  double deadzone_minimum;
};

struct ScalingStatus {
  bool IS_FEASIBLE;
  bool IN_DEADZONE;
  bool IS_SATURATED;
};

class SimpleMixer {
 public:
  SimpleMixer();

  double test;

  void SetMapping(int _index, const Mapping &_mapping);

  void InitializeMixerMatrix(double alpha_f, double alpha_r, double l_hf,
                             double l_hr, double l_vx, double l_vy);

  void SetZeroThrustThreshold(double _v) { zero_throttle_threshold_ = _v; }

  inline double ZeroThrustThreshold() const { return zero_throttle_threshold_; }

  void SetConstantCoefficient(double _v, int idx) {
    thruster_models_[idx].constant_coefficient = _v;
    UpdateMinima();
  }

  inline double ConstantCoefficient(int idx) const {
    return thruster_models_[idx].constant_coefficient;
  }

  void SetLinearCoefficient(double _v, int idx) {
    thruster_models_[idx].linear_coefficient = _v;
    UpdateMinima();
  }

  inline double LinearCoefficient(int idx) const {
    return thruster_models_[idx].linear_coefficient;
  }

  void SetQuadraticCoefficient(double _v, int idx) {
    thruster_models_[idx].quadratic_coefficient = _v;
    ;
    UpdateMinima();
  }

  inline double QuadraticCoefficient(int idx) const {
    return thruster_models_[idx].quadratic_coefficient;
  }

  void SetMinimum(double _v, int idx) {
    thruster_models_[idx].minimum = _v;
    UpdateMinima();
  }

  inline double Minimum(int idx) const { return thruster_models_[idx].minimum; }

  void SetDeadzoneMinimum(double _v, int idx) {
    thruster_models_[idx].deadzone_minimum = _v;
  }

  inline double DeadzoneMinimum(int idx) const {
    return thruster_models_[idx].deadzone_minimum;
  }

  void SetMaxRotationsPerSecond(double _v) { max_rotations_per_second_ = _v; }

  double MaxRotationsPerSecond() const { return max_rotations_per_second_; }

  void SetCompensateDeadZone(bool compensate_deadzone) {
    compensate_deadzone_ = compensate_deadzone;
  }

  void SetWeightingLastInput(double _v) { weighting_last_input_ = _v; }

  void SetScalingSaturationLimitLow(double _v) {
    scaling_saturation_limit_low_ = _v;
  }

  void SetScalingSaturationLimitUp(double _v) {
    scaling_saturation_limit_up_ = _v;
  }

  std::array<double, kOutputChannels> Mix(
      const std::array<double, InputChannels::kCount> &_actuator_controls);

 private:
  void UpdateMinima();  // updates minimum such that there are no issues
                        // possible with the current mapping
  double ThrustToRevsPerSec(double _thrust, int direction);

  double RevsPerSecToThrust(double _thrust, int direction);

  double RevsPerSecToThrust(const ThrusterModel &model, double _thrust);

  bool IsInDeadZone(const double &thruster_value, const double &deadzone_min,
                    const double &deadzone_max);

  template <int size_output>
  void UpdateScaling(const Eigen::Matrix<double, size_output, 1> &thrusters0,
                     double deadzone_min, double deadzone_max,
                     const Eigen::Matrix<double, size_output, 1> &nullspace,
                     double &scaling);

  template <int size_output>
  bool IsSaturated(const Eigen::Matrix<double, size_output, 1> &thrusters,
                   double limit_min, double limit_max);

  template <int size_output>
  Eigen::Matrix<double, size_output, 1> ChooseBetter(
      const std::array<Eigen::Matrix<double, size_output, 1>, 2>
          &thrusters_scaled,
      const Eigen::Matrix<double, size_output, 1> &last_thrusters);

  template <int size_input, int size_output>
  Eigen::Matrix<double, size_output, 1> ResolveDeadzone(
      const Eigen::Matrix<double, size_output, size_input> &M_inv,
      const Eigen::Matrix<double, size_output, 1> &nullspace_vector,
      double deadzone_min, double deadzone_max, double limit_min,
      double limit_max, const Eigen::Matrix<double, size_input, 1> &force_input,
      const Eigen::Matrix<double, size_output, 1> &last_thruster);

  /// @brief per motor mappings of torque/thrust commands
  Mapping mappings_[kOutputChannels];
  Output outputs_[kOutputChannels];
  Eigen::Matrix<double, InputChannels::kCount, kOutputChannels> mixer_matrix_;
  Eigen::Matrix<double, kOutputChannels, InputChannels::kCount>
      mixer_matrix_inverse_;
  Eigen::Matrix<double, kOutputHorizontal, InputChannels::kInputHorizontal>
      mixer_matrix_inverse_horizontal_;
  Eigen::Matrix<double, kOutputVertical, InputChannels::kInputVertical>
      mixer_matrix_inverse_vertical_;
  Eigen::Matrix<double, kOutputHorizontal, 1> nullspace_horizontal_;
  Eigen::Matrix<double, kOutputVertical, 1> nullspace_vertical_;
  Eigen::Matrix<double, kOutputHorizontal, 1> last_output_horizontal_{0.0, 0.0,
                                                                      0.0, 0.0};
  Eigen::Matrix<double, kOutputVertical, 1> last_output_vertical_{0.0, 0.0, 0.0,
                                                                  0.0};
  double scaling_saturation_limit_low_ = -0.8;
  double scaling_saturation_limit_up_ = 0.8;
  double weighting_last_input_ = 0.5;
  const double eps_ = 1e-8;

  double zero_throttle_threshold_;
  std::array<ThrusterModel, 2> thruster_models_;
  bool compensate_deadzone_{false};
  // used as a scaler to normalize the motor command
  double max_rotations_per_second_{1.0};

  double ApplyInput(
      const std::array<double, InputChannels::kCount> &_actuator_controls);

  void ApplyDeadZoneCompensation(
      const std::array<double, InputChannels::kCount> &_actuator_controls,
      double _lower_limit, double _upper_limit);

  void ScaleOutputs(double _scale);

  void ResetOutputs();
};
}  // namespace mixer_bluerov
}  // namespace hippo_control
