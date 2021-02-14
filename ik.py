# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

import math
from leg import Leg
from servo_handler import ServoHandler

class IKSolver:
    STD_THIGH_LEN = 12
    STD_SHIN_LEN = 16
    HIP_OFFSET = 6

    def __init__(self, servo_handler, use_interpolator=False):
        if not type(servo_handler) is ServoHandler:
            raise Exception("Type of servo_handler must be ServoHandler")
        
        self.servo_handler = servo_handler
        self.use_interpolator = use_interpolator

    def solve(self, leg, x, y, z):
        if not type(leg) is Leg:
            raise Exception("Type of leg must be Leg")    

        total_y = (y if not leg.back_axis else -y) + self.HIP_OFFSET
        hip_angle_rad1a = math.atan(total_y/z)
        hip_hypotenuse = total_y / math.sin(hip_angle_rad1a)

        hip_angle_rad1b = math.asin(self.HIP_OFFSET / hip_hypotenuse)
        hip_angle_rad2b = hip_angle_rad1a - hip_angle_rad1b + math.pi/2

        hip_angle_deg = hip_angle_rad2b * 180 / math.pi
        print(hip_angle_deg)

        z1 = self.HIP_OFFSET / math.tan(hip_angle_rad1b)

        shoulder_angle_rad = math.atan(x/z)
        z2 = z / math.cos(shoulder_angle_rad)
        cos_shoulder = (self.STD_THIGH_LEN**2 + z2**2 - self.STD_SHIN_LEN**2) / (2 * self.STD_THIGH_LEN * z2)
        cos_knee = (self.STD_THIGH_LEN**2 + self.STD_SHIN_LEN**2 - z2**2) / (2*self.STD_THIGH_LEN*self.STD_SHIN_LEN)
        shoulder_angle_rad2 = math.acos(cos_shoulder)
        knee_angle_rad = math.acos(cos_knee)

        shoulder_deg = shoulder_angle_rad * 180 / math.pi
        shoulder_deg1 = shoulder_angle_rad2 * 180 / math.pi
        final_shoulder_deg = shoulder_deg + shoulder_deg1 + 90 + leg.shoulder_offset

        knee_deg = knee_angle_rad * 180 / math.pi

        self.servo_handler.write_channel(leg.base_address, hip_angle_deg, leg.scale_factor)
        self.servo_handler.write_channel(leg.base_address + 4, final_shoulder_deg if not leg.right_axis else 180 - final_shoulder_deg, leg.scale_factor)
        self.servo_handler.write_channel(leg.base_address + 8, knee_deg if not leg.right_axis else 180 - knee_deg, leg.scale_factor)