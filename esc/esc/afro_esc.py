# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

import numpy
import smbus2


class AfroESC(object):
    REG_SET_SPEED = 0x00
    REG_GET_SPEED = 0x02
    REG_GET_VBAT = 0x04
    REG_GET_TEMP = 0x06
    REG_GET_ID = 0x08
    ID = 0xAB
    MAX_SPEED = 32767
    INVERT_DIRECTION = (1 << 15)
    ADC_VREF = 5.0
    ADC_RESOLUTION = 1024
    ADC_VBAT_DIV = 6.45
    ADC_VBAT_SCALER = ADC_VREF / ADC_RESOLUTION * ADC_VBAT_DIV

    def __init__(self, bus: smbus2.SMBus, address, pole_pairs=6) -> None:
        """Class to communicate with an AfroESC.

        Args:
            bus (smbus2.SMBus): Existing bus instance
            address (int): Address of the motor. Usually 0x29 + MOTOR_ID
            pole_pairs (int, optional): 6 for Prop NDrive 26-30 800KV. 7 for
                older Turnigy motors. Defaults to 6.
        """
        self._bus = bus
        self.address = address
        self._pole_pairs = pole_pairs

    def init_motor(self):
        self.set_motor_speed(1)
        self.set_motor_speed(0)

    def set_motor_speed(self, speed):
        """Set the motor speed by an arbitrary speed unit.

        This is most likely by far not proportional to the motor's RPM.

        Args:
            speed (_type_): Motor speed.
        """
        speed = min(1.0, max(-1.0, speed))
        speed_val = 0
        if speed < 0:
            speed *= -1
            speed_val += self.INVERT_DIRECTION
        speed_val += int(self.MAX_SPEED * speed)
        # ESC expects MSB. SMBus is LSB. So byte order needs to be inversed.
        self._bus.write_word_data(self.address, self.REG_SET_SPEED,
                                  ((speed_val << 8) & 0xFF00) |
                                  ((speed_val >> 8) & 0x00FF))

    def get_motor_com_counter(self) -> int:
        """Get number of commutations since last request. Will overflow after
        2^16-1 commutations.

        Returns:
            int: Number of commutations since last request.
        """
        data = self._bus.read_i2c_block_data(self.address, self.REG_GET_SPEED,
                                             2)
        return int(data[0] << 8 | data[1])

    def get_motor_rev_counter(self) -> float:
        """Get the number of mechanical revolutions since last request.

        Revolutions are n_commutations/n_poles.

        Returns:
            float: Mechanical revolutions since last request.
        """
        data = self._bus.read_i2c_block_data(self.address, self.REG_GET_SPEED,
                                             2)
        return (data[0] << 8 | data[1]) / self._pole_pairs

    def get_battery_adc(self) -> int:
        data = self._bus.read_i2c_block_data(self.address, self.REG_GET_VBAT, 2)
        return int((data[0] << 8 | data[1]) >> 6)

    def get_battery_voltage(self) -> float:
        data = self._bus.read_i2c_block_data(self.address, self.REG_GET_VBAT, 2)
        return ((data[0] << 8 | data[1]) >> 6) * self.ADC_VBAT_SCALER

    def get_temperature_adc(self) -> int:
        data = self._bus.read_i2c_block_data(self.address, self.REG_GET_TEMP, 2)
        return int(data[0] << 8 | data[1])

    def get_id(self) -> int:
        return self._bus.read_byte_data(self.address, self.REG_GET_ID)

    def verify_id(self) -> bool:
        """Check if ESC responds to ID request with the expected ID.

        You can use this method to check for existence of an ESC.

        Returns:
            bool: True if correct ID was received from ESC.
        """
        id = self.get_id()
        return id == self.ID


def main():
    # pylint: disable=import-outside-toplevel
    import time
    bus = smbus2.SMBus(1)
    esc = AfroESC(bus, 0x29)
    speeds = list(numpy.linspace(0.5, 1, 3))
    print(esc.get_motor_rev_counter())
    esc.init_motor()
    for speed in speeds:
        print("Set speed: {}".format(speed))
        esc.get_motor_rev_counter()
        time1 = time.time()
        loop_start = time.time()
        while time.time() - loop_start < 2.0:
            esc.set_motor_speed(speed)
            time.sleep(0.01)
        cnt = esc.get_motor_rev_counter()
        time2 = time.time()
        print(cnt / (time2 - time1) * 60.0)


if __name__ == "__main__":
    main()
