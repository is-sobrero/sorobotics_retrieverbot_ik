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

leg_fr = Leg(
    base_address=0x16,
    right_axis=True,
    shoulder_offset=10
)

ik = IKSolver(servo)

def seq(start, stop, step=1):
    n = int(round((stop - start)/float(step)))
    if n > 1:
        return([start + step*i for i in range(n+1)])
    elif n == 1:
        return([start])
    else:
        return([])

legs = [leg_fl,  leg_fr, leg_bl, leg_br]

delay_rate = .01
for leg in legs:
    for i in seq(15, 10, step=-0.2):
        ik.solve(leg, 0, 0, i)
        time.sleep(delay_rate)
    for i in seq(10, 15, step=0.2):
        ik.solve(leg, 0, 0, i)
        time.sleep(delay_rate)

'''
for i in seq(0, 5, step=0.1):
        ik.solve(leg_bl, i, 0, 20)
        ik.solve(leg_fl, i, 0, 20)
        ik.solve(leg_br, i, 0, 20)
        time.sleep(delay_rate)

for i in seq(5, -3, step=-0.1):
        ik.solve(leg_bl, i, 0, 20)
        ik.solve(leg_fl, i, 0, 20)
        ik.solve(leg_br, i, 0, 20)
        time.sleep(delay_rate)

for i in seq(-3, 0, step=0.1):
        ik.solve(leg_bl, i, 0, 20)
        ik.solve(leg_fl, i, 0, 20)
        ik.solve(leg_br, i, 0, 20)
        time.sleep(delay_rate)

'''
