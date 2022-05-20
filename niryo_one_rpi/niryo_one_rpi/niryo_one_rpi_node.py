#!/usr/bin/env python

# niryo_one_rpi_node.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#
# All nodes related to Raspberry Pi are running here with only one node
# ( lower RAM usage )
#

import rclpy
import RPi.GPIO as GPIO

from niryo_one_rpi.led_manager import LEDManager
from niryo_one_rpi.fans_manager import FansManager

class NiryoOneRpi:

    def __init__(self,node):
        self.fans_manager = FansManager(node)
        self.led_manager = LEDManager(node)


def main(args = None):
    rclpy.init(args=args)
    node = rclpy.create_node('niryo_one_rpi',allow_undeclared_parameters=True,automatically_declare_parameters_from_overrides=True)
    node.get_logger().info('Created node')
    NiryoOneRpi(node)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
