#include <math.h>
#include <state_estimation/common.h>
#include <state_estimation/interface.h>
#include <state_estimation/util.h>

void Interface::SetImuData(const ImuSample &imu_sample) {
  if (!initialized_) {
    initialized_ = Init(imu_sample.time_us);
  }

  const uint64_t delta_time_us =
      Clip<uint64_t>(imu_sample.time_us - time_last_imu_, 100, 40000);
  time_last_imu_ = imu_sample.time_us;

  if (time_last_imu_ > 0) {
    imu_dt_average_ =
        0.8 * imu_dt_average_ + 0.2 * (double)delta_time_us * 1e-6;
  }
  latest_imu_sample_ = imu_sample;
  // TODO: call the imu downsampler in case it will be implemented
  imu_updated_ = true;
  if (imu_updated_) {
    imu_buffer_.Push(imu_sample);
    imu_sample_delayed_ = imu_buffer_.Oldest();
    min_observation_interval_us_ =
        (imu_sample.time_us - imu_sample_delayed_.time_us) /
        (observation_buffer_length_ - 1);
  }
}

void Interface::SetBaroData(const BaroSample &baro_sample) {
  if (!initialized_) {
    return;
  }

  // allocate more memory if necessary
  if (baro_buffer_.Length() < observation_buffer_length_) {
    // TODO: handle allocation failure
    baro_buffer_.Allocate(observation_buffer_length_);
  }

  // guard against higher update rates than IMU
  if ((baro_sample.time_us - time_last_baro_) > min_observation_interval_us_) {
    time_last_baro_ = baro_sample.time_us;
    BaroSample new_sample;
    new_sample.height = baro_sample.height;
    new_sample.time_us = baro_sample.time_us - settings_.baro_delay_us;
    baro_buffer_.Push(new_sample);

  } else {
    EKF_WARN(
        "Baro sample discarded because interval is smaller than minimum "
        "observation interval.");
  }
}

void Interface::SetVisionData(const VisionSample &vision_sample) {
  if (!initialized_) {
    return;
  }

  if (vision_buffer_.Length() < observation_buffer_length_) {
    // TODO: handle allocation failure
    vision_buffer_.Allocate(observation_buffer_length_);
  }

  if ((vision_sample.time_us - time_last_vision_) <=
      min_observation_interval_us_) {
    EKF_WARN(
        "Vision sample discarded because interval is smaller than minimum "
        "observation interval.");
    return;
  }
  time_last_vision_ = vision_sample.time_us;
  VisionSample new_sample = vision_sample;
  new_sample.time_us -= settings_.vision_delay_us;
  new_sample.time_us -= kFilterUpdatePeriodUs / 2;
  vision_buffer_.Push(new_sample);
}

bool Interface::InitInterface(uint64_t timestamp_us) {
  uint64_t max_time_delay_us = settings_.baro_noise;
  max_time_delay_us =
      std::max<uint64_t>(settings_.vision_delay_us, max_time_delay_us);

  imu_buffer_length_ = max_time_delay_us / kFilterUpdatePeriodUs + 1;

  const uint64_t filter_delay_us = max_time_delay_us * 1.5;
  observation_buffer_length_ =
      filter_delay_us / settings_.min_observation_interval_us + 1;
  observation_buffer_length_ =
      std::min<uint64_t>(observation_buffer_length_, imu_buffer_length_);
  EKF_INFO("Observation buffer timespan: %lu us",
           observation_buffer_length_ * settings_.min_observation_interval_us);
  EKF_INFO("Imu buffer timespan: %lu us",
           kFilterUpdatePeriodUs * imu_buffer_length_);
  if (!imu_buffer_.Allocate(imu_buffer_length_) ||
      !output_buffer_.Allocate(imu_buffer_length_)) {
    return false;
  }

  // TODO: why is this timestamp necessary?
  imu_sample_delayed_.time_us = timestamp_us;
  return true;
}
