# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

import math
from leg import Leg
from servo_handler import ServoHandler

class ServoDomainError(Exception, leg):
    pass

class IKSolver:
    STD_THIGH_LEN = 12
    STD_SHIN_LEN = 14
    HIP_OFFSET = 5.5

    def __init__(self, servo_handler, use_interpolator=False):
        if not type(servo_handler) is ServoHandler:
            raise Exception("Type of servo_handler must be ServoHandler")
        
        self.servo_handler = servo_handler
        self.use_interpolator = use_interpolator

    def solve(self, leg, x, y, z):
        if not type(leg) is Leg:
            raise TypeError("Type of leg must be Leg")   
        # P(x,y,z) O(0,0,0)
        # length from O to P projection on YZ plane
        dyz = math.sqrt(x**2 + z**2)
        # total height of the leg on th YZ plane
        hyz = math.sqrt(dyz**2 - self.HIP_OFFSET**2) 

        fy = -y if leg.right_axis else y
        gamma_yz = -math.atan(fy/z)
        gamma_hip = -math.atan(self.HIP_OFFSET/hyz)
        gamma = gamma_yz - gamma_hip + math.pi/2 # final gamma angle in radiants

        beta_offset = math.atan(x/hyz)
        new_z = hyz/math.cos(beta_offset)

        beta_cos = (self.STD_THIGH_LEN**2 + new_z**2 - self.STD_SHIN_LEN**2 ) / (2*self.STD_THIGH_LEN*new_z)
        beta = math.acos(beta_cos) + math.pi/2 + beta_offset

        alpha_cos = (self.STD_THIGH_LEN**2 + self.STD_SHIN_LEN**2 - new_z**2) / (2*self.STD_THIGH_LEN*self.STD_SHIN_LEN)
        alpha = math.acos(alpha_cos)
        
        alfa_deg = alpha * 180 / math.pi
        beta_deg = beta * 180 / math.pi
        gamma_deg = gamma * 180 / math.pi

        final_alfa = leg.alfa_offset + (alfa_deg if not leg.right_axis else 180 - alfa_deg)
        final_beta = leg.beta_offset + (beta_deg if not leg.right_axis else 180 - beta_deg)
        final_gamma = leg.gamma_offset + (gamma_deg if not leg.invert_shoulder else 180 - gamma_deg)

        if final_alfa < 0:
            raise ServoDomainError("Alfa angle of leg is not positive", leg)   
        if final_beta < 0:
            raise ServoDomainError("Beta angle of leg is not positive", leg)   
        if final_gamma < 0:
            raise ServoDomainError("Gamma angle of leg is not positive", leg)   

        self.servo_handler.write_channel(leg.base_address, final_gamma, leg.scale_factor)
        self.servo_handler.write_channel(leg.base_address + 4, final_beta, leg.scale_factor)
        self.servo_handler.write_channel(leg.base_address + 8, final_alfa, leg.scale_factor)