#pragma once

#include <fcntl.h>
#include <stdint.h>

#include <array>
#include <string>

namespace hardware {
namespace barometer {

namespace Oversampling {
static constexpr int k256 = 0;
static constexpr int k512 = 0;
static constexpr int k1024 = 0;
static constexpr int k2048 = 0;
static constexpr int k4096 = 0;
static constexpr int k8192 = 0;
}  // namespace Oversampling

enum class Model { _02BA = 0, _30BA };

class MS5837 {
 public:
  enum class Status { kOk = 0, kCrcError, kIOError, kResetError };
  MS5837();
  ~MS5837();
  static constexpr uint8_t kAddress = 0x76;
  static constexpr uint8_t kRegisterReset = 0x1E;
  static constexpr uint8_t kRegisterAdcRead = 0x00;
  static constexpr uint8_t kRegisterPromRead = 0xA0;
  static constexpr uint8_t kRegisterConvertTemperature = 0x40;
  static constexpr uint8_t kRegisterConvertPressure = 0x50;

  bool Open(std::string _device_name = "/dev/i2c-1");
  Status Init();
  bool Reset();
  bool Read(int oversampling);
  /// @brief Pressure in [Pa]
  double Pressure() const { return pressure_; }
  double PressureCompensated() const { return pressure_compensated_; }
  /// @brief Temperature in deg Celsius.
  double Temperature() const { return temperature_; }
  double TemperatureCompensated() const { return temperature_compensated_; }

private:
  struct Compensation {
    uint32_t raw_pressure;     /// pressure as read from the sensor
    uint32_t raw_temperature;  /// temperature as read from the sensor
    int32_t delta_temperature; /// difference between actual and reference
                               /// temperature
    int32_t temperature_cK;    /// actual temperature in centi Kelvin
    int64_t temperature_correction;
    int64_t offset;
    int64_t offset_correction;
    int64_t sensitivity; /// sensitivity at current temperature
    int64_t sensitivity_correction;
    int32_t pressure_cBar;
  };
  uint8_t Crc4(std::array<uint16_t, 8> &_data);
  void ApplyCalibration();
  void LowTemperatureCompensation(Compensation &_compensation);
  void HighTemperatureCompensation(Compensation &_compensation);
  void SecondOrderCompensation(Compensation &_compensation);
  std::string device_name_;
  int file_handle_;

  double pressure_;
  double temperature_;
  double pressure_compensated_;
  double temperature_compensated_;

  /// @brief MSB first
  std::array<uint8_t, 3> temperature_raw_;
  /// @brief MSB first
  std::array<uint8_t, 3> pressure_raw_;

  std::array<uint16_t, 6> calibration_;
  std::array<uint16_t, 8> prom_;
};

}  // namespace barometer
}  // namespace hardware
