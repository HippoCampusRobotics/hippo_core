// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

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

MS5837::Status MS5837::Read(MS5837::Oversampling oversampling) {
  using namespace std::chrono;
  double tmp = 2.2e-6 * (0x01 << (8 + oversampling));
  auto conversion_time = round<nanoseconds>(duration<double>(tmp));

  std::array<uint8_t, 0> data;
  i2c::WriteData(file_handle_, kAddress,
                 kRegisterConvertTemperature + 2 * oversampling, data);
  std::this_thread::sleep_for(conversion_time);
  if (!i2c::ReadData(file_handle_, kAddress, kRegisterAdcRead,
                     temperature_raw_)) {
    return Status::kIOError;
  }

  i2c::WriteData(file_handle_, kAddress,
                 kRegisterConvertPressure + 2 * oversampling, data);
  std::this_thread::sleep_for(conversion_time);
  if (!i2c::ReadData(file_handle_, kAddress, kRegisterAdcRead, pressure_raw_)) {
    return Status::kIOError;
  }
  ApplyCalibration();
  return Status::kOk;
}

void MS5837::ApplyCalibration() {
  Compensation c;
  c.raw_temperature = temperature_raw_[0] << 16 | temperature_raw_[1] << 8 |
                      temperature_raw_[2];
  c.raw_pressure =
      pressure_raw_[0] << 16 | pressure_raw_[1] << 8 | pressure_raw_[2];
  c.delta_temperature = c.raw_temperature - ((int32_t)prom_[5] << 8);

  c.temperature_cK =
      2000 + (((int64_t)c.delta_temperature * (int64_t)prom_[6]) >> 23);
  temperature_ = c.temperature_cK * 0.01;

  c.offset =
      prom_[2] * (0x01 << 16) + prom_[4] * c.delta_temperature / (0x01 << 7);
  c.sensitivity =
      prom_[1] * (0x01 << 15) + prom_[3] * c.delta_temperature / (0x01 << 8);
  c.pressure_cBar =
      (c.raw_pressure * c.sensitivity / (0x01 << 21) - c.offset) / (0x01 << 13);
  pressure_ = c.pressure_cBar * 1e-4 * 1e5;
  SecondOrderCompensation(c);
}

void MS5837::SecondOrderCompensation(Compensation &_c) {
  if (temperature_ < 20.0) {
    LowTemperatureCompensation(_c);
  } else {
    HighTemperatureCompensation(_c);
  }
  _c.offset_correction = _c.offset - _c.offset_correction;
  _c.sensitivity_correction = _c.sensitivity - _c.sensitivity_correction;

  temperature_compensated_ =
      (static_cast<int64_t>(_c.temperature_cK) - _c.temperature_correction) *
      0.01;
  int64_t tmp = static_cast<int64_t>(_c.raw_pressure);
  int64_t pressure_cBar;
  pressure_cBar =
      (((tmp * _c.sensitivity_correction) >> 21) - _c.offset_correction) >> 13;
  pressure_compensated_ = pressure_cBar * 1e-4 * 1e5;
}

void MS5837::LowTemperatureCompensation(Compensation &_c) {
  int64_t tmp_int64;
  tmp_int64 = static_cast<int64_t>(_c.delta_temperature);
  _c.temperature_correction = (3 * tmp_int64) >> 33;

  tmp_int64 = static_cast<int64_t>(_c.temperature_cK) - 2000;
  _c.offset_correction = (3 * tmp_int64 * tmp_int64) >> 1;
  _c.sensitivity_correction = (5 * tmp_int64 * tmp_int64) >> 3;

  if (_c.temperature_cK >= -1500) {
    return;
  }
  tmp_int64 = static_cast<int64_t>(_c.temperature_cK) + 1500;
  _c.offset_correction += 7 * tmp_int64 * tmp_int64;
  _c.sensitivity_correction += 4 * tmp_int64 * tmp_int64;
}

void MS5837::HighTemperatureCompensation(Compensation &_c) {
  int64_t tmp_int64;

  tmp_int64 = static_cast<int64_t>(_c.delta_temperature);
  _c.temperature_correction = (2 * tmp_int64 * tmp_int64) >> 37;
  tmp_int64 = static_cast<int64_t>(_c.temperature_cK) - 2000;
  _c.offset_correction = (tmp_int64 * tmp_int64) >> 4;
  _c.sensitivity_correction = 0;
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
