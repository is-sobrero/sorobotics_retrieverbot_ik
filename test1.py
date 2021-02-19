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
    shoulder_offset=-10,
    offsets=[0,-10,-7],
    back_axis=True
)

leg_fl = Leg(
    base_address=0x3a,
    shoulder_offset=5
)

leg_br = Leg(
    base_address=0x22,
    scale_factor=ServoHandler.TOWERPRO_SF,
    right_axis=True,
    shoulder_offset=3,
    back_axis=True
)

leg_fr = Leg(
    base_address=0x16,
    right_axis=True,
    shoulder_offset=10,
    offsets=[0,0,5],
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

delay_rate = .01
steps = 1
for i in seq(15, 20, step=0.2):
        ik.solve(leg_bl, 0, 5.5, i)
        ik.solve(leg_fl, 0, 5.5, i)
        ik.solve(leg_br, 0, -5.5, i)
        ik.solve(leg_fr, 0, -5.5, i)
        time.sleep(delay_rate)
for x in range(0,steps):
    for i in seq(20, 10, step=-0.2):
        ik.solve(leg_bl, 0, 5.5, i)
        ik.solve(leg_fl, 0, 5.5, i)
        ik.solve(leg_br, 0, -5.5, i)
        ik.solve(leg_fr, 0, -5.5, i)
        time.sleep(delay_rate)
    for i in seq(10, 20, step=0.2):
        ik.solve(leg_bl, 0, 5.5, i)
        ik.solve(leg_fl, 0, 5.5, i)
        ik.solve(leg_br, 0, -5.5, i)
        ik.solve(leg_fr, 0, -5.5, i)
        time.sleep(delay_rate)

for i in seq(20, 15, step=-0.2):
        ik.solve(leg_bl, 0, 5.5, i)
        ik.solve(leg_fl, 0, 5.5, i)
        ik.solve(leg_br, 0, -5.5, i)
        ik.solve(leg_fr, 0, -5.5, i)
        time.sleep(delay_rate)

for i in seq(5.5, -2, step=-0.2):
        ik.solve(leg_bl, 0, i, 15)
        ik.solve(leg_fl, 0, i, 15)
        ik.solve(leg_br, 0, -11+i, 15)
        ik.solve(leg_fr, 0, -11+i, 15)
        time.sleep(delay_rate)
time.sleep(1)
for i in seq(-2, 5.5, step=0.2):
        ik.solve(leg_bl, 0, i, 15)
        ik.solve(leg_fl, 0, i, 15)
        ik.solve(leg_br, 0, -11+i, 15)
        ik.solve(leg_fr, 0, -11+i, 15)
        time.sleep(delay_rate)
time.sleep(1)
for i in seq(5.5, 13, step=0.2):
        ik.solve(leg_bl, 0, i, 15)
        ik.solve(leg_fl, 0, i, 15)
        ik.solve(leg_br, 0, -11+i, 15)
        ik.solve(leg_fr, 0, -11+i, 15)
        time.sleep(delay_rate)
time.sleep(1)
for i in seq(13, 5.5, step=-0.2):
        ik.solve(leg_bl, 0, i, 15)
        ik.solve(leg_fl, 0, i, 15)
        ik.solve(leg_br, 0, -11+i, 15)
        ik.solve(leg_fr, 0, -11+i, 15)
        time.sleep(delay_rate)
time.sleep(1)

for i in seq(0, 7, step=0.1):
        ik.solve(leg_bl, i, 5.5, 15)
        ik.solve(leg_fl, i, 5.5, 15)
        ik.solve(leg_br, i, -5.5, 15)
        ik.solve(leg_fr, i, -5.5, 15)
        time.sleep(delay_rate)
time.sleep(1)

for i in seq(7, -3, step=-0.1):
        ik.solve(leg_bl, i, 5.5, 15)
        ik.solve(leg_fl, i, 5.5, 15)
        ik.solve(leg_br, i, -5.5, 15)
        ik.solve(leg_fr, i, -5.5, 15)
        time.sleep(delay_rate)
time.sleep(1)

for i in seq(-3, 0, step=0.1):
        ik.solve(leg_bl, i, 5.5, 15)
        ik.solve(leg_fl, i, 5.5, 15)
        ik.solve(leg_br, i, -5.5, 15)
        ik.solve(leg_fr, i, -5.5, 15)
        time.sleep(delay_rate)


