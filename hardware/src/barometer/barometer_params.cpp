#include "barometer.hpp"
#include <hippo_common/param_utils.hpp>

namespace hardware {
namespace barometer {

void Barometer::InitParams() {
  HIPPO_COMMON_DECLARE_PARAM_READONLY(device);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(update_period_ms);
}

} // namespace barometer
} // namespace hardware
