#pragma once

#include <cmath>

namespace hippo_control {

template <typename T>
int sgn(T value) {
  return (T(0) < value) - (value < T(0));
}

class ThrusterModel {
 public:
  double ThrustToRotationsPerSecond(double _thrust) {
    double thrust_sign = sgn(_thrust);
    _thrust *= thrust_sign;
    if (linear_coefficient_ == 0.0) {
      if (quadratic_coefficient_ == 0.0) {
        return 0.0;
      }
      // F(n) = an^2 + c
      return thrust_sign *
             sqrt((_thrust - constant_coefficient_) / quadratic_coefficient_);
    }

    if (quadratic_coefficient_ == 0.0) {
      // F(n) = bn + c
      return thrust_sign * (_thrust - constant_coefficient_) /
             linear_coefficient_;
    }

    // F(n) = an^2 + bn + c
    return thrust_sign *
           (-1.0 * linear_coefficient_ +
            sqrt(4.0 * quadratic_coefficient_ * _thrust +
                 linear_coefficient_ * linear_coefficient_ -
                 4.0 * quadratic_coefficient_ * constant_coefficient_)) /
           (2.0 * quadratic_coefficient_);
  }
  inline double ThrustToEscCommand(double thrust) {
    return ThrustToRotationsPerSecond(thrust) / max_revs_per_second_;
  }

 private:
  double constant_coefficient_{-0.8576};
  double linear_coefficient_{0.1086};
  double quadratic_coefficient_{0.003313};
  double max_revs_per_second_{50.0};
};
}  // namespace hippo_control
