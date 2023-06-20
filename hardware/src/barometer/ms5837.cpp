#include "hardware/barometer/ms5837.hpp"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <chrono>
#include <thread>

#include "hardware/i2c/i2c.hpp"

namespace hardware {
namespace barometer {

MS5837::MS5837() {}

bool MS5837::Open(std::string _device_name) {
  device_name_ = _device_name;
  file_handle_ = open(device_name_.c_str(), O_RDWR);
  if (file_handle_ < 0) {
    return false;
  }
  return true;
}

MS5837::Status MS5837::Init() {
  if (!Reset()) {
    return Status::kResetError;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  for (std::size_t i = 0; i < prom_.size() - 1; ++i) {
    if (!i2c::ReadWord(file_handle_, kAddress, kRegisterPromRead + 2 * i,
                       prom_[i])) {
      return Status::kIOError;
    }
  }

  uint8_t crc = (prom_[0] & 0xF000) >> 12;
  if (crc != Crc4(prom_)) {
    return Status::kCrcError;
  }
  return Status::kOk;
}

bool MS5837::Reset() {
  std::array<uint8_t, 0> data;
  return i2c::WriteData(file_handle_, kAddress, kRegisterReset, data);
}

bool MS5837::Read(int oversampling) {
  using namespace std::chrono;
  double tmp = 2.2e-6 * (0x01 << (8 + oversampling));
  auto conversion_time = round<nanoseconds>(duration<double>(tmp));

  std::array<uint8_t, 0> data;
  i2c::WriteData(file_handle_, kAddress,
                 kRegisterConvertTemperature + 2 * oversampling, data);
  std::this_thread::sleep_for(conversion_time);
  if (!i2c::ReadData(file_handle_, kAddress, kRegisterAdcRead,
                     temperature_raw_)) {
    return false;
  }

  i2c::WriteData(file_handle_, kAddress,
                 kRegisterConvertPressure + 2 * oversampling, data);
  std::this_thread::sleep_for(conversion_time);
  if (!i2c::ReadData(file_handle_, kAddress, kRegisterAdcRead, pressure_raw_)) {
    return false;
  }
  ApplyCalibration();
  return true;
}

void MS5837::ApplyCalibration() {
  uint32_t raw_temperature = temperature_raw_[0] << 16 |
                             temperature_raw_[1] << 8 | temperature_raw_[2];
  uint32_t raw_pressure =
      pressure_raw_[0] << 16 | pressure_raw_[1] << 8 | pressure_raw_[2];
  int32_t dT = raw_temperature - prom_[5] * (0x01 << 8);

  int32_t temperature_cK = 2000 + dT * prom_[6] / (0x01 << 23);
  temperature_ = temperature_cK / 100.0;

  int64_t offset = prom_[2] * (0x01 << 16) + prom_[4] * dT / (0x01 << 7);
  int64_t sensitivity = prom_[1] * (0x01 << 15) + prom_[3] * dT / (0x01 << 8);
  int32_t pressure =
      (raw_pressure * sensitivity / (0x01 << 21) - offset) / (0x01 << 13);
  pressure_ = pressure * 0.01 * 10e5;

  // TODO: second order compensation
  if (temperature_ < 20.0) {
    // TODO
  } else {
    // TODO
  }
}

uint8_t MS5837::Crc4(std::array<uint16_t, 8> &prom) {
  bool toggle{true};
  uint16_t crc = 0;

  // remove crc nibble
  prom[0] &= 0x0FFF;
  prom[prom.size() - 1] = 0;

  for (std::size_t i = 0; i < prom.size() * sizeof(uint16_t); ++i) {
    if (toggle) {
      crc ^= (uint16_t)(prom[i >> 1] >> 8);
    } else {
      crc ^= (uint16_t)(prom[i >> 1] & 0x00FF);
    }
    toggle = !toggle;
    for (int j = 8; j > 0; --j) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x3000;
      } else {
        crc = (crc << 1);
      }
    }
  }
  crc = (crc >> 12) & 0x000F;
  return crc ^ 0x00;
}

}  // namespace barometer
}  // namespace hardware
