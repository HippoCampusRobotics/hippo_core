#include <state_estimation/ekf.h>
#include <state_estimation/util.h>

#include <cmath>
#include <iostream>

bool Ekf::Init(uint64_t timestamp_us) {
  bool ret = InitInterface(timestamp_us);
  Reset();
  return ret;
}

void Ekf::Reset() {
  EKF_INFO("Resetting");
  state_.velocity.setZero();
  state_.position.setZero();
  state_.delta_angle_bias.setZero();
  state_.delta_velocity_bias.setZero();
  state_.orientation.setIdentity();

  dt_average_ = kFilterUpdatePeriodUs * 1e-6;

  output_new_.orientation.setIdentity();
  output_new_.position.setZero();
  output_new_.velocity.setZero();
  delta_angle_correction_.setZero();

  accel_magnitude_filtered_ = 0.0;
  gyro_magnitude_filtered_ = 0.0;
  prev_delta_velocity_bias_var_.setZero();

  imu_updated_ = false;
  filter_initialized_ = false;
}

bool Ekf::InitFilter() {
  const ImuSample &imu_init = imu_buffer_.Newest();
  if (imu_init.delta_velocity_dt < 1e-4 || imu_init.delta_angle_dt < 1e-4) {
    EKF_WARN(
        "IMU update frequency too high. If this happens regularly it indicates "
        "an error.");
    return false;
  }

  if (is_first_imu_sample_) {
    acceleration_filter_.Reset(imu_init.delta_velocity /
                               imu_init.delta_velocity_dt);
    gyro_filter_.Reset(imu_init.delta_angle / imu_init.delta_angle_dt);
    is_first_imu_sample_ = false;
  } else {
    acceleration_filter_.Update(imu_init.delta_velocity /
                                imu_init.delta_velocity_dt);
    gyro_filter_.Update(imu_init.delta_angle / imu_init.delta_angle_dt);
  }

  if (baro_buffer_.PopFirstOlderThan(imu_sample_delayed_.time_us,
                                     &baro_sample_delayed_)) {
    if (baro_sample_delayed_.time_us != 0) {
      // TODO: change this automatic height offset to some more reasonable
      // method. For example setting it manually.
      baro_height_offset_ = settings_.baro_height_offset;
      // if (baro_counter_ == 0) {
      //   baro_height_offset_ = baro_sample_delayed_.height;
      // } else {
      //   baro_height_offset_ =
      //       0.9 * baro_height_offset_ + 0.1 * baro_sample_delayed_.height;
      // }

      baro_counter_++;
    } else {
      EKF_WARN("Delayed baro sample has invalid timestamp!");
    }
  }

  if (baro_counter_ < observation_buffer_length_) {
    EKF_INFO("Waiting for more barometric data. Having (%d/%d).", baro_counter_,
             observation_buffer_length_);
    return false;
  } else {
    EKF_INFO_ONCE("Got sufficient amount of baro samples.");
  }

  if (!InitTilt()) {
    EKF_WARN("Tilt could not be initialized");
    return false;
  }
  // TODO: reset fusion times to latest imu

  InitCovariance();
  AlignOutputFilter();
  return true;
}

bool Ekf::InitTilt() {
  const double acceleration_norm = acceleration_filter_.State().norm();
  const double gyro_norm = gyro_filter_.State().norm();

  if (acceleration_norm < 0.8 * kGravity ||
      acceleration_norm > 1.2 * kGravity || gyro_norm > 15.0 * kPi / 180.0) {
    return false;
  }

  const Eigen::Vector3d gravity_body =
      acceleration_filter_.State().normalized();
  // TODO: check for correctness. also see:
  // https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
  const double pitch = asin(gravity_body(0));
  const double roll = atan2(-gravity_body(1), gravity_body(2));
  EKF_INFO("Initializing tilt with roll=%.2f, pitch=%.2f", pitch * 180.0 / M_PI,
           roll * 180 / M_PI);
  state_.orientation = EulerToQuaternion(roll, pitch, 0.0);
  state_.orientation.normalize();
  R_to_earth_ = state_.orientation.matrix();
  control_status_.flags.vision_orientation = true;
  // control_status_.flags.tilt_align = true;
  return true;
}

bool Ekf::Update() {
  bool updated = false;
  if (!filter_initialized_) {
    filter_initialized_ = InitFilter();
    if (!filter_initialized_) {
      return false;
    }
    EKF_INFO("Initialized!");
  }

  if (imu_updated_) {
    // TODO: does it make sense? In theo original implementation only the imu
    // down-sampler sets the imu_updated_ state.
    updated = true;
    PredictState();
    PredictCovariance();
    UpdateSensorFusion();
    CalculateOutputState(latest_imu_sample_);
  }
  imu_updated_ = false;
  return updated;
}

void Ekf::PredictState() {
  Eigen::Vector3d delta_angle_corrected =
      imu_sample_delayed_.delta_angle - state_.delta_angle_bias;
  // TODO: check if correction necessary.
  // see:
  // https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/ekf.cpp#L265

  // TODO: is this correct?
  // see: https://stanford.edu/class/ee267/lectures/lecture10.pdf p.26
  // but this is not the way the original code calculates it.
  const Eigen::Quaterniond delta_quat = QuaternionFromDeltaAngle(
      delta_angle_corrected, imu_sample_delayed_.delta_angle_dt);

  state_.orientation = (state_.orientation * delta_quat).normalized();
  R_to_earth_ = state_.orientation.toRotationMatrix();

  const Eigen::Vector3d delta_velocity_corrected =
      imu_sample_delayed_.delta_velocity - state_.delta_velocity_bias;
  const Eigen::Vector3d delta_velocity_corrected_earth =
      R_to_earth_ * delta_velocity_corrected;

  const Eigen::Vector3d velocity_last = state_.velocity;
  // TODO: check if delta velocity corrected still behaves weird.
  state_.velocity += delta_velocity_corrected_earth;
  // state_.velocity += imu_sample_delayed_.delta_velocity;
  // compensate gravity
  state_.velocity(2) -= kGravity * imu_sample_delayed_.delta_velocity_dt;

  // trapezoidal integration
  state_.position += (velocity_last + state_.velocity) *
                     imu_sample_delayed_.delta_velocity_dt * 0.5;

  ConstrainStates();

  double dt = 0.5 * (imu_sample_delayed_.delta_velocity_dt +
                     imu_sample_delayed_.delta_angle_dt);
  dt = Clip<double>(dt, 0.5 * kFilterUpdatePeriodUs * 1e-6,
                    2.0 * kFilterUpdatePeriodUs * 1e-6);

  dt_average_ = 0.99 * dt_average_ + 0.01 * dt;

  if (imu_sample_delayed_.delta_angle_dt >
      0.25 * kFilterUpdatePeriodUs * 1e-6) {
    angular_rate_delayed_raw_ =
        imu_sample_delayed_.delta_angle / imu_sample_delayed_.delta_angle_dt;
  }
}
/**
 * @brief Compute the output state, i.e. the estimated state at the current time
 * horizon.
 *
 * The state of the delayed time horizon — estimated by the EKF — gets projected
 * forward in time based on the current IMU samples.
 *
 * @param imu_sample Latest IMU sample
 */
void Ekf::CalculateOutputState(const ImuSample &imu_sample) {
  const double dt_scale_correction = imu_dt_average_ / dt_average_;
  const Eigen::Vector3d delta_angle(
      imu_sample.delta_angle - state_.delta_angle_bias * dt_scale_correction +
      delta_angle_correction_);
  const double spin_delta_angle_D =
      delta_angle.dot(Eigen::Vector3d(R_to_earth_now_.row(2)));
  yaw_delta_ef_ += spin_delta_angle_D;

  // apply some basic low pass filtering to smooth the yaw rate.
  yaw_rate_lpf_ef_ = 0.95 * yaw_rate_lpf_ef_ +
                     0.05 * spin_delta_angle_D / imu_sample.delta_angle_dt;
  // The delta quaternion by which the output needs to be rotated.
  const Eigen::Quaterniond delta_quat =
      QuaternionFromDeltaAngle(delta_angle, imu_sample.delta_angle_dt);
  output_new_.time_us = imu_sample.time_us;
  // rotate the current orientation by the delta quaternion
  output_new_.orientation = output_new_.orientation * delta_quat;
  // make sure the orientation is a unit quaternion
  output_new_.orientation.normalize();
  R_to_earth_now_ = output_new_.orientation.toRotationMatrix();

  const Eigen::Vector3d delta_velocity_body{imu_sample.delta_velocity -
                                            state_.delta_velocity_bias *
                                                dt_scale_correction};
  Eigen::Vector3d delta_velocity_earth{R_to_earth_now_ * delta_velocity_body};

  delta_velocity_earth(2) -= kGravity * imu_sample.delta_velocity_dt;

  if (imu_sample.delta_velocity_dt > 1e-4) {
    velocity_derivative_ =
        delta_velocity_earth * (1.0 / imu_sample.delta_velocity_dt);
  }
  const Eigen::Vector3d velocity_last{output_new_.velocity};
  output_new_.velocity += delta_velocity_earth;
  // trapezoidal integration
  const Eigen::Vector3d delta_position =
      (output_new_.velocity + velocity_last) *
      (0.5 * imu_sample.delta_velocity_dt);
  output_new_.position += delta_position;

  if (imu_sample.delta_angle_dt > 1e-4) {
    const Eigen::Vector3d angular_rate =
        imu_sample.delta_angle * (1.0 / imu_sample.delta_angle_dt);
    const Eigen::Vector3d velocity_imu_rel_body =
        angular_rate.cross(settings_.imu_position_body);
    velocity_rel_imu_body_enu_ = R_to_earth_now_ * velocity_imu_rel_body;
  }

  if (imu_updated_) {
    // store all the output states in the buffer, so we can apply corrections as
    // soon as the delayed EKF has a new estimate
    output_buffer_.Push(output_new_);
    const OutputSample &output_delayed = output_buffer_.Oldest();
    const Eigen::Quaterniond q_error(
        (state_.orientation.inverse() * output_delayed.orientation)
            .normalized());
    const double scalar = (q_error.w() >= 0.0) ? -2.0 : 2.0;
    const Eigen::Vector3d delta_angle_error{
        scalar * q_error.x(), scalar * q_error.y(), scalar * q_error.z()};
    const double time_delay =
        std::max((imu_sample.time_us - imu_sample_delayed_.time_us) * 1e-6,
                 imu_dt_average_);
    const double attitude_gain = 0.5 * imu_dt_average_ / time_delay;

    delta_angle_correction_ = delta_angle_error * attitude_gain;
    output_tracking_error_(0) = delta_angle_error.norm();
    const double velocity_gain =
        dt_average_ /
        Clip<double>(settings_.velocity_time_constant, dt_average_, 10.0);
    const double position_gain =
        dt_average_ /
        Clip<double>(settings_.position_time_constant, dt_average_, 10.0);

    const Eigen::Vector3d velocity_error(state_.velocity -
                                         output_delayed.velocity);
    const Eigen::Vector3d position_error(state_.position -
                                         output_delayed.position);
    output_tracking_error_(1) = velocity_error.norm();
    output_tracking_error_(2) = position_error.norm();

    velocity_error_integral_ += velocity_error;
    const Eigen::Vector3d velocity_correction =
        velocity_error * velocity_gain +
        velocity_error_integral_ * square(velocity_gain) * 0.1;

    position_error_integral_ += position_error;
    const Eigen::Vector3d position_correction =
        position_error * position_gain +
        position_error_integral_ * square(position_gain) * 0.1;
    CorrectOutputBuffer(velocity_correction, position_correction);
  }
}

void Ekf::CorrectOutputBuffer(const Eigen::Vector3d &velocity_correction,
                              const Eigen::Vector3d &position_correction) {
  for (int index = 0; index < output_buffer_.Length(); ++index) {
    output_buffer_[index].velocity += velocity_correction;
    output_buffer_[index].position += position_correction;
  }
  output_new_ = output_buffer_.Newest();
}

void Ekf::Fuse(const StateVectord &K, double innovation) {
  // if you ask yourself why all the signs are negative: because the innovation
  // was computed h(state) - measurement instead of the other way round.
  Eigen::Vector4d tmp = K.block<4, 1>(StateIndex::qw, 0) * innovation;
  state_.orientation.w() -= tmp(0);
  state_.orientation.x() -= tmp(1);
  state_.orientation.y() -= tmp(2);
  state_.orientation.z() -= tmp(3);
  state_.orientation.normalize();
  state_.velocity -= K.block<3, 1>(StateIndex::velocity_x, 0) * innovation;
  state_.position -= K.block<3, 1>(StateIndex::position_x, 0) * innovation;
  state_.delta_angle_bias -=
      K.block<3, 1>(StateIndex::delta_angle_bias_x, 0) * innovation;
  state_.delta_velocity_bias -=
      K.block<3, 1>(StateIndex::delta_velocity_bias_x, 0) * innovation;
}

void Ekf::FuseOrientation() {
  Eigen::Quaterniond orientation;
  if (control_status_.flags.vision_orientation) {
    orientation = vision_sample_delayed_.orientation;
  } else {
    return;
  }
  const Eigen::Quaterniond q_innov =
      Eigen::Quaterniond{state_.orientation.w() - orientation.w(),
                         state_.orientation.x() - orientation.x(),
                         state_.orientation.y() - orientation.y(),
                         state_.orientation.z() - orientation.z()}
          .normalized();
  Eigen::Vector4d innovation = {q_innov.w(), q_innov.x(), q_innov.y(),
                                q_innov.z()};
  Eigen::Vector4d innovation_var = {P_(0, 0) + 0.01, P_(1, 1) + 0.01,
                                    P_(2, 2) + 0.01, P_(3, 3) + 0.01};
  StateVectord K_fusion;
  StateMatrixd KHP;
  for (int i = 0; i < 4; ++i) {
    for (int row = 0; row < StateIndex::NumStates; ++row) {
      K_fusion(row) = P_(row, i) / innovation_var(i);
    }

    for (int row = 0; row < StateIndex::NumStates; ++row) {
      for (int col = 0; col < StateIndex::NumStates; ++col) {
        KHP(row, col) = K_fusion(row) * P_(i, col);
      }
    }

    bool healthy = true;
    for (int index = 0; index < StateIndex::NumStates; ++index) {
      if (P_(index, index) < KHP(index, index)) {
        healthy = false;
        P_.UncorrelateCovarianceSetVariance<1>(index, 1.0);
      }
    }

    if (healthy) {
      P_ -= KHP;
      FixCovarianceErrors(true);
      Fuse(K_fusion, innovation(i));
    } else {
      EKF_WARN("Unhealthy innovation: q%d", i);
    }
  }
}

void Ekf::FuseHeading() {
  double yaw_var;
  double measured_heading;

  if (control_status_.flags.vision_yaw) {
    yaw_var = vision_sample_delayed_.angular_variance;
  } else {
    yaw_var = 0.01;
  }

  R_to_earth_ = state_.orientation.matrix();
  if (ShouldUse321RotationSequence(R_to_earth_)) {
    const double predicted_heading = Euler321Yaw(R_to_earth_);
    if (control_status_.flags.vision_yaw) {
      measured_heading = Euler321Yaw(vision_sample_delayed_.orientation);
    } else {
      measured_heading = predicted_heading;
    }

    bool fuse_zero_innovation = false;
    last_static_yaw_ = predicted_heading;
    FuseYaw321(measured_heading, yaw_var, fuse_zero_innovation);
  } else {
    const double predicted_heading = Euler312Yaw(R_to_earth_);
    if (control_status_.flags.vision_yaw) {
      measured_heading = Euler312Yaw(vision_sample_delayed_.orientation);
    } else {
      measured_heading = predicted_heading;
    }

    bool fuse_zero_innovation = false;
    last_static_yaw_ = predicted_heading;
    FuseYaw312(measured_heading, yaw_var, fuse_zero_innovation);
  }
}

// copy of
// https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/mag_fusion.cpp#L420
void Ekf::FuseYaw321(double yaw, double yaw_var, bool zero_innovation) {
  // assign intermediate state variables
  const double q0 = state_.orientation.w();
  const double q1 = state_.orientation.x();
  const double q2 = state_.orientation.y();
  const double q3 = state_.orientation.z();

  const double R_YAW = std::max(yaw_var, 1.0e-4);
  const double measurement = WrapPi(yaw);

  // calculate 321 yaw observation matrix
  // choose A or B computational paths to avoid singularity in derivation at
  // +-90 degrees yaw
  bool canUseA = false;
  const double SA0 = 2 * q3;
  const double SA1 = 2 * q2;
  const double SA2 = SA0 * q0 + SA1 * q1;
  const double SA3 = square(q0) + square(q1) - square(q2) - square(q3);
  double SA4, SA5_inv;
  if (square(SA3) > 1e-6) {
    SA4 = 1.0 / square(SA3);
    SA5_inv = square(SA2) * SA4 + 1;
    canUseA = abs(SA5_inv) > 1e-6;
  }

  bool canUseB = false;
  const double SB0 = 2 * q0;
  const double SB1 = 2 * q1;
  const double SB2 = SB0 * q3 + SB1 * q2;
  const double SB4 = square(q0) + square(q1) - square(q2) - square(q3);
  double SB3, SB5_inv;
  if (square(SB2) > 1e-6) {
    SB3 = 1.0 / square(SB2);
    SB5_inv = SB3 * square(SB4) + 1;
    canUseB = abs(SB5_inv) > 1e-6;
  }

  Eigen::Vector4d H_YAW;

  if (canUseA && (!canUseB || abs(SA5_inv) >= abs(SB5_inv))) {
    const double SA5 = 1.0 / SA5_inv;
    const double SA6 = 1.0 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_YAW(0) = SA5 * (SA0 * SA6 - SA8 * q0);
    H_YAW(1) = SA5 * (SA1 * SA6 - SA8 * q1);
    H_YAW(2) = SA5 * (SA1 * SA7 + SA9 * q1);
    H_YAW(3) = SA5 * (SA0 * SA7 + SA9 * q0);
  } else if (canUseB && (!canUseA || abs(SB5_inv) > abs(SA5_inv))) {
    const double SB5 = 1.0 / SB5_inv;
    const double SB6 = 1.0 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_YAW(0) = -SB5 * (SB0 * SB6 - SB8 * q3);
    H_YAW(1) = -SB5 * (SB1 * SB6 - SB8 * q2);
    H_YAW(2) = -SB5 * (-SB1 * SB7 - SB9 * q2);
    H_YAW(3) = -SB5 * (-SB0 * SB7 - SB9 * q3);
  } else {
    return;
  }

  // calculate the yaw innovation and wrap to the interval between +-pi
  double innovation;
  if (zero_innovation) {
    innovation = 0.0;
  } else {
    innovation =
        WrapPi(atan2(R_to_earth_(1, 0), R_to_earth_(0, 0)) - measurement);
  }

  // define the innovation gate size
  double innov_gate = std::max(settings_.heading_innovation_gate, 1.0);

  // Update the quaternion states and covariance matrix
  UpdateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

// copy of
// https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/mag_fusion.cpp#L500
void Ekf::FuseYaw312(double yaw, double yaw_var, bool zero_innovation) {
  const double q0 = state_.orientation.w();
  const double q1 = state_.orientation.x();
  const double q2 = state_.orientation.y();
  const double q3 = state_.orientation.z();

  const double R_YAW = std::max<double>(yaw_var, 1.0e-4);
  const double measurement = WrapPi(yaw);

  // calculate 312 yaw observation matrix
  // choose A or B computational paths to avoid singularity in derivation at
  // +-90 degrees yaw
  bool canUseA = false;
  const double SA0 = 2 * q3;
  const double SA1 = 2 * q2;
  const double SA2 = SA0 * q0 - SA1 * q1;
  const double SA3 = square(q0) - square(q1) + square(q2) - square(q3);
  double SA4, SA5_inv;
  if (square(SA3) > 1e-6) {
    SA4 = 1.0 / square(SA3);
    SA5_inv = square(SA2) * SA4 + 1;
    canUseA = abs(SA5_inv) > 1e-6f;
  }

  bool canUseB = false;
  const double SB0 = 2 * q0;
  const double SB1 = 2 * q1;
  const double SB2 = -SB0 * q3 + SB1 * q2;
  const double SB4 = -square(q0) + square(q1) - square(q2) + square(q3);
  double SB3, SB5_inv;
  if (square(SB2) > 1e-6) {
    SB3 = 1.0 / square(SB2);
    SB5_inv = SB3 * square(SB4) + 1;
    canUseB = abs(SB5_inv) > 1e-6f;
  }

  Eigen::Vector4d H_YAW;

  if (canUseA && (!canUseB || abs(SA5_inv) >= abs(SB5_inv))) {
    const double SA5 = 1.0 / SA5_inv;
    const double SA6 = 1.0 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_YAW(0) = SA5 * (SA0 * SA6 - SA8 * q0);
    H_YAW(1) = SA5 * (-SA1 * SA6 + SA8 * q1);
    H_YAW(2) = SA5 * (-SA1 * SA7 - SA9 * q1);
    H_YAW(3) = SA5 * (SA0 * SA7 + SA9 * q0);
  } else if (canUseB && (!canUseA || abs(SB5_inv) > abs(SA5_inv))) {
    const double SB5 = 1.0 / SB5_inv;
    const double SB6 = 1.0 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_YAW(0) = -SB5 * (-SB0 * SB6 + SB8 * q3);
    H_YAW(1) = -SB5 * (SB1 * SB6 - SB8 * q2);
    H_YAW(2) = -SB5 * (-SB1 * SB7 - SB9 * q2);
    H_YAW(3) = -SB5 * (SB0 * SB7 + SB9 * q3);
  } else {
    return;
  }

  double innovation;
  if (zero_innovation) {
    innovation = 0.0f;
  } else {
    // calculate the the innovation and wrap to the interval between +-pi
    innovation =
        WrapPi(atan2(-R_to_earth_(0, 1), R_to_earth_(1, 1)) - measurement);
  }

  // define the innovation gate size
  double innov_gate = std::max<double>(settings_.heading_innovation_gate, 1.0);

  // Update the quaternion states and covariance matrix
  UpdateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

void Ekf::UpdateQuaternion(const double innovation, const double variance,
                           const double gate_sigma,
                           const Eigen::Vector4d &yaw_jacobian) {
  heading_innovation_ = variance;
  for (int row = 0; row < 4; ++row) {
    double tmp = 0.0;
    for (int col = 0; col < 4; ++col) {
      tmp += P_(row, col) * yaw_jacobian(col);
    }
    heading_innovation_var_ += yaw_jacobian(row) * tmp;
  }
  double heading_innovation_var_inv;

  if (heading_innovation_var_ >= variance) {
    fault_status_.flags.bad_heading = false;
    heading_innovation_var_inv = 1.0 / heading_innovation_var_;
  } else {
    fault_status_.flags.bad_heading = true;
    InitCovariance();
    // TODO: log this!
    return;
  }
  StateVectord Kfusion;
  for (int row = 0; row < 16; ++row) {
    for (int col = 0; col < 4; ++col) {
      Kfusion(row) += P_(row, col) * yaw_jacobian(col);
    }
    Kfusion(row) *= heading_innovation_var_inv;
  }
  yaw_test_ratio_ =
      square(innovation) / (square(gate_sigma) * heading_innovation_var_);
  if (yaw_test_ratio_ > 1.0) {
    innovation_check_status_.flags.reject_yaw = true;
    if (!control_status_.flags.in_air &&
        IsTimedOut(time_last_in_air_us_, (uint64_t)5e6)) {
      double gate_limit = sqrt((square(gate_sigma) * heading_innovation_var_));
      heading_innovation_ = Clip(innovation, -gate_limit, gate_limit);
      ResetYawGyroBiasCov();
    } else {
      return;
    }
  } else {
    innovation_check_status_.flags.reject_yaw = false;
    heading_innovation_ = innovation;
  }

  StateMatrixd KHP;
  double KH[4];
  for (int row = 0; row < StateIndex::NumStates; row++) {
    KH[0] = Kfusion(row) * yaw_jacobian(0);
    KH[1] = Kfusion(row) * yaw_jacobian(1);
    KH[2] = Kfusion(row) * yaw_jacobian(2);
    KH[3] = Kfusion(row) * yaw_jacobian(3);

    for (int column = 0; column < StateIndex::NumStates; column++) {
      double tmp = KH[0] * P_(0, column);
      tmp += KH[1] * P_(1, column);
      tmp += KH[2] * P_(2, column);
      tmp += KH[3] * P_(3, column);
      KHP(row, column) = tmp;
    }
  }
  const bool healthy = CheckAndFixCovarianceUpdate(KHP);
  fault_status_.flags.bad_heading = !healthy;
  if (healthy) {
    P_ -= KHP;
    FixCovarianceErrors(true);
    Fuse(Kfusion, heading_innovation_);
  }
}

bool Ekf::FuseHorizontalPosition(const Eigen::Vector3d &innovation,
                                 const Eigen::Vector2d &innovation_gate,
                                 const Eigen::Vector3d &observation_var,
                                 Eigen::Vector3d &innovation_var,
                                 Eigen::Vector2d &test_ratio,
                                 bool inhibit_gate = false) {
  innovation_var(0) =
      P_(StateIndex::position_x, StateIndex::position_x) + observation_var(0);
  innovation_var(1) =
      P_(StateIndex::position_y, StateIndex::position_y) + observation_var(1);
  test_ratio(0) = std::max(
      square(innovation(0)) / (square(innovation_gate(0)) * innovation_var(0)),
      square(innovation(1)) / (square(innovation_gate(0)) * innovation_var(1)));

  const bool innovation_check_pass = test_ratio(0) <= 1.0;
  if (innovation_check_pass &&
      test_ratio(0) > square(100.0 / innovation_gate(0))) {
    if (inhibit_gate && test_ratio(0) > square(100.0 / innovation_gate(0))) {
      return false;
    }
    time_last_vision_ = time_last_imu_;
    FuseVelocityPositionHeight(innovation(0), innovation_var(0), 3);
    FuseVelocityPositionHeight(innovation(1), innovation_var(1), 4);
    return true;
  } else {
    EKF_INFO("Refusing horizontal position.");
    innovation_check_status_.flags.reject_horizontal_position = true;
    return false;
  }
}

bool Ekf::FuseVerticalPosition(const Eigen::Vector3d &innovation,
                               const Eigen::Vector2d &innovation_gate,
                               const Eigen::Vector3d &observation_var,
                               Eigen::Vector3d &innovation_var,
                               Eigen::Vector2d &test_ratio) {
  innovation_var(2) = P_(9, 9) + observation_var(2);
  test_ratio(1) =
      square(innovation(2)) / (square(innovation_gate(1)) * innovation_var(2));
  vertical_position_innovation_ratio_ = innovation(2) / sqrt(innovation_var(2));
  vertical_position_fuse_attempt_time_us_ = time_last_imu_;
  bool innovation_check_pass = test_ratio(1) <= 1.0;

  double innovation_tmp;
  // TODO: check for bad vertical acceleration
  // see:
  // https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/vel_pos_fusion.cpp#L160
  innovation_tmp = innovation(2);
  if (innovation_check_pass) {
    time_last_height_fuse_ = time_last_imu_;
    innovation_check_status_.flags.reject_vertical_position = false;
    FuseVelocityPositionHeight(innovation_tmp, innovation_var(2), 5);
    return true;
  } else {
    innovation_check_status_.flags.reject_vertical_position = true;
    return false;
  }
}

void Ekf::FuseVelocityPositionHeight(const double innovation,
                                     const double innovation_var,
                                     const int observation_index) {
  StateVectord K_fusion;
  // offset the index by the quaternion, so 0 starts at the velocity.
  const int state_index = observation_index + StateIndex::velocity_x;
  for (int row = 0; row < StateIndex::NumStates; ++row) {
    K_fusion(row) = P_(row, state_index) / innovation_var;
  }
  StateMatrixd KHP;
  for (int row = 0; row < StateIndex::NumStates; ++row) {
    for (int col = 0; col < StateIndex::NumStates; ++col) {
      KHP(row, col) = K_fusion(row) * P_(state_index, col);
    }
  }
  bool healthy = true;
  for (int i = 0; i < StateIndex::NumStates; ++i) {
    if (P_(i, i) < KHP(i, i)) {
      P_.row(i).setConstant(0.0);
      P_.col(i).setConstant(0.0);
      healthy = false;
    }
  }
  SetVelocityPositionFaultStatus(observation_index, !healthy);
  if (healthy) {
    P_ -= KHP;
    FixCovarianceErrors(true);
    Fuse(K_fusion, innovation);
  } else {
    EKF_WARN("Unhealthy observation: %d", state_index);
  }
}

void Ekf::SetVelocityPositionFaultStatus(const int index, const bool is_bad) {
  switch (index) {
    case 0:
      fault_status_.flags.bad_velocity_x = is_bad;
      break;
    case 1:
      fault_status_.flags.bad_velocity_y = is_bad;
      break;
    case 2:
      fault_status_.flags.bad_velocity_z = is_bad;
      break;
    case 3:
      fault_status_.flags.bad_position_x = is_bad;
      break;
    case 4:
      fault_status_.flags.bad_position_y = is_bad;
      break;
    case 5:
      fault_status_.flags.bad_position_z = is_bad;
      break;
    default:
      break;
  }
}

void Ekf::UpdateDeadreckoningStatus() {}

// see
// https://github.com/PX4/PX4-Autopilot/blob/8cc6d02af324ba713beb80b374236f70ac7f0a9a/src/modules/ekf2/EKF/ekf_helper.cpp#L1073

void Ekf::UpdateSensorFusion() {
  control_status_prev_.value = control_status_.value;
  if (!control_status_.flags.tilt_align) {
    const Eigen::Vector3d angle_err_vec_var = CalcRotationVectorVariances();

    if ((angle_err_vec_var(0) + angle_err_vec_var(1)) <
        square(3.0 / 180 * kPi)) {
      control_status_.flags.tilt_align = true;
      EKF_INFO("Tilt aligned.");
    } else {
      EKF_DEBUG("Error Vec Var: %.6lf",
                angle_err_vec_var(0) + angle_err_vec_var(1));
    }
  }

  const BaroSample &baro_init = baro_buffer_.Newest();
  baro_height_faulty_ = !IsRecent(baro_init.time_us, 2 * kBaroMaxIntervalUs);
  delta_time_baro_us_ = baro_sample_delayed_.time_us;
  baro_data_ready_ = baro_buffer_.PopFirstOlderThan(imu_sample_delayed_.time_us,
                                                    &baro_sample_delayed_);

  if (baro_data_ready_) {
    delta_time_baro_us_ = baro_sample_delayed_.time_us - delta_time_baro_us_;
  }
  vision_data_ready_ = vision_buffer_.PopFirstOlderThan(
      imu_sample_delayed_.time_us, &vision_sample_delayed_);

  UpdateHeightSensorTimeout();
  UpdateHeightFusion();
  UpdateVisionFusion();
  UpdateDeadreckoningStatus();
}

void Ekf::UpdateHeightSensorTimeout() {
  CheckVerticalAccelHealth();
  const bool continuous_bad_accel = IsTimedOut(
      time_good_vertical_accel_us_, settings_.bad_accel_reset_delay_us);
  const bool timed_out = IsTimedOut(time_last_height_fuse_, (uint64_t)5e6);
  if (timed_out || continuous_bad_accel) {
    bool request_height_reset = false;

    if (control_status_.flags.baro_height) {
      // const bool prev_bad_vertical_accel =
      //     IsRecent(time_bad_vertical_accel_us_, kBadAccelProbation);
      if (!baro_height_faulty_) {
        request_height_reset = true;
      }
    } else if (control_status_.flags.vision_height) {
      const VisionSample &vision_init = vision_buffer_.Newest();
      const bool vision_data_available =
          IsRecent(vision_init.time_us, 2 * kVisionMaxIntervalUs);
      if (vision_data_available) {
        request_height_reset = true;
      } else if (!baro_height_faulty_) {
        StartBaroHeightFusion();
        request_height_reset = true;
      }
    }

    if (request_height_reset) {
      ResetHeight();
      time_last_height_fuse_ = time_last_imu_;
    }
  }
}

void Ekf::UpdateHeightFusion() {
  bool fuse_height = false;
  switch (settings_.height_mode) {
    case HeightMode::kBaro:
      if (baro_data_ready_ && !baro_height_faulty_) {
        StartBaroHeightFusion();
        fuse_height = true;
      }
      break;
    case HeightMode::kVision:
      if (!control_status_.flags.vision_height &&
          IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        fuse_height = true;
        SetControlVisionHeight();
        // vision height flag is not yet set. Reset the height until the flag is
        // set.
        ResetHeight();
      }
      if (control_status_.flags.vision_height && vision_data_ready_) {
        fuse_height = true;
      } else if (control_status_.flags.baro_height && baro_data_ready_ &&
                 !baro_height_faulty_) {
        fuse_height = true;
      }
      break;
    default:
      EKF_WARN("No input for height fusion selected.");
      break;
  }
  UpdateBaroHeightOffset();

  if (fuse_height) {
    if (control_status_.flags.baro_height) {
      Eigen::Vector2d baro_innovation_gate;
      Eigen::Vector3d baro_observation_var;
      // TODO: check the signs of the following equation
      baro_height_innovation_(2) =
          state_.position(2) -
          (baro_sample_delayed_.height + baro_height_offset_);
      baro_observation_var(2) =
          square(std::max<double>(settings_.baro_noise, 0.01));
      baro_innovation_gate(1) =
          std::max<double>(settings_.baro_innovation_gate, 1.0);

      FuseVerticalPosition(baro_height_innovation_, baro_innovation_gate,
                           baro_observation_var, baro_height_innovation_var_,
                           baro_height_test_ratio_);
    } else if (control_status_.flags.vision_height) {
      Eigen::Vector2d vision_innovation_gate;
      Eigen::Vector3d vision_observation_var;

      vision_position_innovation_(2) =
          state_.position(2) - vision_sample_delayed_.position(2);
      vision_observation_var(2) = std::max<double>(
          vision_sample_delayed_.position_variance(2), square(0.01));
      vision_innovation_gate(1) =
          std::max<double>(settings_.vision_innovation_gate, 1.0);
      FuseVerticalPosition(vision_position_innovation_, vision_innovation_gate,
                           vision_observation_var,
                           vision_position_innovation_var_,
                           vision_position_test_ratio_);
    } else {
      EKF_WARN("Trying to fuse height, but no input choice flag is set.");
    }
  }
}

void Ekf::UpdateVisionFusion() {
  if (vision_data_ready_) {
    EKF_INFO("Vision data ready to fuse.");
    if (control_status_.flags.tilt_align && control_status_.flags.yaw_align &&
        !control_status_.flags.vision_position) {
      if (IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        StartVisionPositionFusion();
      }
    }

    if (!control_status_.flags.vision_yaw && control_status_.flags.tilt_align) {
      if (IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        if (ResetYawToVision()) {
          control_status_.flags.yaw_align = true;
          StartVisionYawFusion();
        }
      }
    }

    if (control_status_.flags.vision_position) {
      EKF_INFO("Fusing vision position.");
      Eigen::Vector3d vision_observation_var;
      Eigen::Vector2d vision_innovation_gate;

      Eigen::Vector3d position_measurement = vision_sample_delayed_.position;
      Eigen::Matrix3d position_var = Eigen::DiagonalMatrix<double, 3>(
          vision_sample_delayed_.position_variance);
      vision_position_innovation_(0) =
          state_.position(0) - position_measurement(0);
      vision_position_innovation_(1) =
          state_.position(1) - position_measurement(1);
      vision_observation_var(0) =
          std::max<double>(position_var(0, 0), square(0.01));
      vision_observation_var(1) =
          std::max<double>(position_var(1, 1), square(0.01));

      if (IsTimedOut(time_last_horizontal_position_fuse_,
                     settings_.reset_timeout_max_us)) {
        ResetVelocity();
        ResetHorizontalPosition();
      }
      vision_innovation_gate(0) =
          std::max<double>(settings_.vision_innovation_gate, 1.0);
      FuseHorizontalPosition(vision_position_innovation_,
                             vision_innovation_gate, vision_observation_var,
                             vision_position_innovation_var_,
                             vision_position_test_ratio_);
    }
    if (control_status_.flags.vision_orientation) {
      FuseOrientation();
    } else if (control_status_.flags.vision_yaw) {
      FuseHeading();
    }
  } else if ((control_status_.flags.vision_position ||
              control_status_.flags.vision_yaw) &&
             IsTimedOut(time_last_vision_, settings_.reset_timeout_max_us)) {
    StopVisionFusion();
  }
}
