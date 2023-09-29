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

#pragma once

#include <stdint.h>
#include <algorithm>

#include "esc.h"
class AfroESC : public ESCBase {
 private:
  static constexpr int kRegSetSpeed = 0x00;
  static constexpr int kRegGetSpeed = 0x02;
  static constexpr int kRegGetVbat = 0x04;
  static constexpr int kRegGetTemp = 0x06;
  static constexpr int kRegGetId = 0x08;
  static constexpr int kId = 0xAB;
  static constexpr int kMaxSpeed = INT16_MAX;
  static constexpr int kInvertDirection = (1 << 15);
  static constexpr double kAdcVref = 5.0;
  static constexpr int kAdcResolution = 1024;
  static constexpr double kAdcVbatVoltageDivider = 6.45;
  static constexpr double kAdcVbatScaler =
      kAdcVref / kAdcResolution * kAdcVbatVoltageDivider;

  int i2c_handle_;
  int i2c_address_;
  int pole_pairs_;
  bool available_{false};
  int index_;
  double throttle_;
  bool in_use_{false};
  double revolution_count_;
  int commutation_count_;
  int battery_adc_;
  double battery_voltage_;

  EscRetCode ReadWordData(int _reg_address, int &_data);
  EscRetCode WriteWordData(int _reg_address, int data);

 public:
  AfroESC(const int _i2c_handle = 0, const int _i2c_address = 0,
          const int _pole_pairs = 6);
  EscRetCode WriteThrottle();
  EscRetCode Reset(int _i2c_handle, int _i2c_address);
  inline void SetThrottle(double _throttle) {
    throttle_ = std::clamp(_throttle, -1.0, 1.0);
  }
  EscRetCode UpdateRevolutionCount();
  int GetCommutationCount();
  double GetRevolutionCount();
  EscRetCode UpdateBatteryAdc();
  double GetBatteryVoltage();
  EscRetCode UpdateTemperatureAdc();
  EscRetCode ReadId(int &_id);
  EscRetCode VerifyID(bool &_is_ok);
  bool available();
  void SetAvailable(bool available);
  void SetIndex(int index);
  int index();
  void SetAddress(int _address);
  int address();
  bool InUse();
  void SetInUse(bool _in_use);
};
