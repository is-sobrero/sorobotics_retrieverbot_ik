# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

import time

class ServoHandler:
    MIN_PRESCALER = 100
    MAX_PRESCALER = 500
    ANGLE_RANGE = 180
    PRESCALER_STEP = (MAX_PRESCALER - MIN_PRESCALER) / ANGLE_RANGE
    TOWERPRO_SF = 2/3
    DMG_SF = 1

    def __init__(self, board_address):
        try:
            import smbus
            self.mb_address = board_address
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(self.mb_address, 0xfe, 0x79)
            self.bus.write_byte_data(self.mb_address, 0x00, 0x20)
            time.sleep(.25)
        except ImportError:
            print("SMBus cannot be imported, presuming you're running on a simulated environment, servo_handler has not been initialized")
    
    def write_channel(self, base_address, angle, scale_factor=1):
        prescaler = int(self.MIN_PRESCALER + (self.PRESCALER_STEP * angle * scale_factor))
        self.bus.write_word_data(self.mb_address, base_address, 0)
        self.bus.write_word_data(self.mb_address, base_address + 2, prescaler)