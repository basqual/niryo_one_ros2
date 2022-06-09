#!/usr/bin/env python

# fans_manager.py
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

from time import sleep
import rclpy
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

FAN_1_GPIO = 27
FAN_2_GPIO = 23


class FansManager:

    def __init__(self,node):
        self.node = node
        self.setup_fans()
        self.learning_mode_on = True
        # Activate fans for 5 seconds to give an audio signal to the user
        self.set_fans(True)
        sleep(5)
        self.set_fans(not self.learning_mode_on)
        namespace = node.get_namespace() if node.get_namespace() != "/" else ""
        self.learning_mode_subscriber = self.node.create_subscription(Bool,namespace + '/niryo_one/learning_mode', self.callback_learning_mode,10)

    def setup_fans(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FAN_1_GPIO, GPIO.OUT)
        GPIO.setup(FAN_2_GPIO, GPIO.OUT)
        sleep(0.05)
        GPIO.output(FAN_1_GPIO, GPIO.HIGH)
        self.node.get_logger().info("------ RPI FANS SETUP OK ------")

    def set_fans(self,activate):
        if activate:
            GPIO.output(FAN_2_GPIO, GPIO.HIGH)
        else:
            GPIO.output(FAN_2_GPIO, GPIO.LOW)

    def callback_learning_mode(self, msg):
        if msg.data != self.learning_mode_on:
            self.learning_mode_on = msg.data
            self.set_fans(not self.learning_mode_on)
