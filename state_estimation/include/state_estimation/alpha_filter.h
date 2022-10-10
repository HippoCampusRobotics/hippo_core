#pragma once
#include <limits>

template <typename T>
class AlphaFilter {
 public:
  AlphaFilter() = default;
  explicit AlphaFilter(double alpha) : alpha_(alpha) {}

  ~AlphaFilter() = default;

  void SetAlpha(double alpha) { alpha_ = alpha; }

  void Reset(const T &sample) { state_ = sample; }

  const T &Update(const T &sample) {
    state_ = UpdateCalculation(sample);
    return state_;
  }

  const T &State() const { return state_; }

  void SetParameters(double sample_interval, double time_constant) {
    const double denom = time_constant + sample_interval;
    if (denom > std::numeric_limits<double>::epsilon()) {
      SetAlpha(sample_interval / denom);
    }
  }

 protected:
  T UpdateCalculation(const T &sample) {
    return (1.0 - alpha_) * state_ + alpha_ * sample;
  }

  double alpha_{0.0};
  T state_{};
};
