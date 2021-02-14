# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

from ik import IKSolver
from servo_handler import ServoHandler
from leg import Leg
import time

BACK_LEFT_ADDRESS = 0x2E
FRONT_LEFT_ADDRESS = 0x3A
BACK_RIGHT_ADDRESS = 0x22

servo = ServoHandler(0x40)

leg_bl = Leg(
    base_address=0x2e, 
    scale_factor=ServoHandler.TOWERPRO_SF,
    shoulder_offset=-10
)

leg_fl = Leg(
    base_address=0x3a,
    shoulder_offset=10
)

leg_br = Leg(
    base_address=0x22,
    scale_factor=ServoHandler.TOWERPRO_SF,
    right_axis=True,
    shoulder_offset=3
)

ik = IKSolver(servo, use_interpolator=True)

steps = [
    [(-5,0,10), (-5,0,10)],
    [(0,0,15), (0,0,15)],
]

for positions in steps:
    ik.solve(leg_bl, positions[0][0], positions[0][1], positions[0][2])
    ik.solve(leg_fl, positions[1][0], positions[1][1], positions[1][2])
    ik.solve(leg_br, positions[1][0], positions[1][1], positions[1][2])
    time.sleep(1)

