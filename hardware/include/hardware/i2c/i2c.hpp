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

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <array>

namespace hardware {
namespace i2c {

template <std::size_t N>
bool ReadData(int file_handle, uint8_t address, uint8_t register_addr,
              std::array<uint8_t, N> &_data) {
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data msgset[1];

  _data[0] = register_addr;

  msgs[0].addr = address;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = _data.data();

  msgs[1].addr = address;
  msgs[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msgs[1].len = N;
  msgs[1].buf = _data.data();

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = sizeof(msgs) / sizeof(msgs[0]);

  if (ioctl(file_handle, I2C_RDWR, msgset) < 0) {
    return false;
  }
  return true;
}

inline bool ReadWord(int file_handle, uint8_t address, uint8_t register_addr,
                     uint16_t &_data) {
  std::array<uint8_t, 2> buf;
  if (!ReadData(file_handle, address, register_addr, buf)) {
    return false;
  }
  _data = static_cast<uint16_t>(buf[0]) << 8 | buf[1];
  return true;
}

template <std::size_t N>
bool WriteData(int file_handle, uint8_t address, uint8_t register_addr,
               const std::array<uint8_t, N> &_data) {
  uint8_t buf[N + 1];
  struct i2c_msg msg;
  struct i2c_rdwr_ioctl_data ioctl_data;

  buf[0] = register_addr;
  for (std::size_t i = 0; i < N; ++i) {
    buf[i + 1] = _data[i];
  }
  msg.addr = address;
  msg.flags = 0;
  msg.len = N + 1;
  msg.buf = buf;

  ioctl_data.msgs = &msg;
  ioctl_data.nmsgs = 1;

  if (ioctl(file_handle, I2C_RDWR, &ioctl_data) < 0) {
    return false;
  }
  return true;
}

}  // namespace i2c
}  // namespace hardware
