# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

class Leg:
    def __init__(self, base_address, right_axis=False, scale_factor=1, shoulder_offset=0):
        self.base_address = base_address
        self.right_axis = right_axis
        self.scale_factor = scale_factor
        self.shoulder_offset = shoulder_offset