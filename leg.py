# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.
import uuid 

class Leg:
    def __init__(self, base_address=0, right_axis=False, scale_factor=1, shoulder_offset=0, back_axis=False, offsets=[0,0,0]):
        self.base_address = base_address
        self.right_axis = right_axis
        self.scale_factor = scale_factor
        self.shoulder_offset = shoulder_offset
        self.leg_id = uuid.uuid4()
        self.back_axis = back_axis
        self.alfa_offset = offsets[0]
        self.beta_offset = offsets[1]
        self.gamma_offset = offsets[2]
        self.invert_shoulder = (back_axis and not right_axis) or (not back_axis and right_axis)