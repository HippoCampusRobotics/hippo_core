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

#include "afro_esc.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <algorithm>  // used for min max functions

AfroESC::AfroESC(const int _i2c_handle, const int _i2c_address,
                 const int _pole_pairs)
    : available_(false),
      in_use_(false),
      revolution_count_(0.0),
      commutation_count_(0) {
  i2c_handle_ = _i2c_handle;
  i2c_address_ = _i2c_address;
  pole_pairs_ = _pole_pairs;
}

EscRetCode AfroESC::WriteWordData(int _reg_address, int _data) {
  uint8_t buf[3];
  struct i2c_msg msg;
  struct i2c_rdwr_ioctl_data ioctl_data;

  buf[0] = static_cast<uint8_t>(_reg_address);
  buf[1] = static_cast<uint8_t>((_data >> 8) & 0xFF);
  buf[2] = static_cast<uint8_t>(_data & 0xFF);
  msg.addr = i2c_address_;
  msg.flags = 0;
  msg.len = sizeof(buf) / sizeof(buf[0]);
  msg.buf = buf;

  ioctl_data.msgs = &msg;
  ioctl_data.nmsgs = 1;

  if (ioctl(i2c_handle_, I2C_RDWR, &ioctl_data) < 0) {
    return EscRetCode::kIOError;
  }
  return EscRetCode::kOk;
}

EscRetCode AfroESC::ReadWordData(int _reg_address, int &_data) {
  uint8_t buf[2];
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data msgset[1];

  buf[0] = static_cast<uint8_t>(_reg_address);

  msgs[0].addr = i2c_address_;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = buf;

  msgs[1].addr = i2c_address_;
  msgs[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msgs[1].len = sizeof(buf) / sizeof(buf[0]);
  msgs[1].buf = buf;

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = sizeof(msgs) / sizeof(msgs[0]);

  if (ioctl(i2c_handle_, I2C_RDWR, msgset) < 0) {
    return EscRetCode::kIOError;
  }
  _data = buf[0] << 8 | buf[1];
  return EscRetCode::kOk;
}

EscRetCode AfroESC::Reset(int _i2c_handle, int _i2c_address) {
  i2c_handle_ = _i2c_handle;
  i2c_address_ = _i2c_address;
  available_ = false;
  in_use_ = false;
  throttle_ = 0.0;
  battery_adc_ = 0;
  battery_voltage_ = 0.0;
  commutation_count_ = 0;
  revolution_count_ = 0.0;
  return EscRetCode::kOk;
}

EscRetCode AfroESC::WriteThrottle() {
  double throttle = throttle_;
  int speed_data = 0;
  if (throttle < 0) {
    throttle = -1 * throttle_;
    speed_data += kInvertDirection;
  }

  speed_data += (int)(kMaxSpeed * throttle);
  return WriteWordData(kRegSetSpeed, speed_data);
}

EscRetCode AfroESC::UpdateRevolutionCount() {
  EscRetCode status;
  status = ReadWordData(kRegGetSpeed, commutation_count_);
  if (status != EscRetCode::kOk) {
    return status;
  }
  revolution_count_ = static_cast<double>(commutation_count_) / pole_pairs_;
  return EscRetCode::kOk;
}

int AfroESC::GetCommutationCount() { return commutation_count_; }

double AfroESC::GetRevolutionCount() { return revolution_count_; }

EscRetCode AfroESC::UpdateBatteryAdc() {
  EscRetCode status;
  status = ReadWordData(kRegGetVbat, battery_adc_);
  if (status != EscRetCode::kOk) {
    return status;
  }
  // weird bit-shifting needed due to 10bit adc resolution
  battery_adc_ >>= 6;
  battery_voltage_ = battery_adc_ * kAdcVbatScaler;
  return EscRetCode::kOk;
}

double AfroESC::GetBatteryVoltage() { return battery_voltage_; }
EscRetCode AfroESC::UpdateTemperatureAdc() { return EscRetCode::kOk; }

EscRetCode AfroESC::ReadId(int &_id) {
  uint8_t buf[1];
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data msgset[1];

  buf[0] = kRegGetId;

  msgs[0].addr = i2c_address_;
  msgs[0].flags = 0;
  msgs[0].len = sizeof(buf) / sizeof(buf[0]);
  msgs[0].buf = buf;

  msgs[1].addr = i2c_address_;
  msgs[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msgs[1].len = sizeof(buf) / sizeof(buf[0]);
  msgs[1].buf = buf;

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = sizeof(msgs) / sizeof(msgs[0]);
  if (ioctl(i2c_handle_, I2C_RDWR, msgset) < 0) {
    return EscRetCode::kIOError;
  }
  _id = static_cast<int>(buf[0]);
  return EscRetCode::kOk;
}

EscRetCode AfroESC::VerifyID(bool &_is_ok) {
  int id;
  EscRetCode status = ReadId(id);
  if (status != EscRetCode::kOk) {
    SetAvailable(false);
    return status;
  }
  SetAvailable(kId == id);
  _is_ok = kId == id;
  return EscRetCode::kOk;
}

bool AfroESC::available() { return available_; }
void AfroESC::SetAvailable(bool available) { available_ = available; }
void AfroESC::SetIndex(int index) { index_ = index; }
int AfroESC::index() { return index_; }
void AfroESC::SetAddress(int _address) { i2c_address_ = _address; }
int AfroESC::address() { return i2c_address_; }
bool AfroESC::InUse() { return in_use_; }
void AfroESC::SetInUse(bool _in_use) { in_use_ = _in_use; }
