#!/usr/bin/env python

# led_manager.py
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

import rclpy
import RPi.GPIO as GPIO
from time import sleep

from std_msgs.msg import Empty
from std_msgs.msg import Bool

from niryo_one_msgs.msg import HardwareStatus
from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import SetLeds

LED_GPIO_R = 18
LED_GPIO_G = 24
LED_GPIO_B = 22

LED_OFF = 0
LED_BLUE = 1
LED_GREEN = 2
LED_BLUE_GREEN = 3
LED_RED = 4
LED_PURPLE = 5
LED_RED_GREEN = 6
LED_WHITE = 7

class LedState:
    def __init__(self):
        pass

    SHUTDOWN = 1
    HOTSPOT = 2
    HW_ERROR = 3
    OK = 4
    WAIT_HOTSPOT = 5


class LEDManager:
    def __init__(self,node):
        self.node = node
        # Set warning false, and don't cleanup LED GPIO after exit
        # So the LED will be red only after the Rpi is shutdown
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(LED_GPIO_R, GPIO.OUT)
        GPIO.setup(LED_GPIO_G, GPIO.OUT)
        GPIO.setup(LED_GPIO_B, GPIO.OUT)

        sleep(0.1)
        self.state = LedState.OK
        self.set_led_from_state(dxl_leds=True)

        namespace = node.get_namespace() if node.get_namespace() != "/" else ""

        self.set_led_state_server = node.create_service(SetInt,'/niryo_one/rpi/set_led_state', self.callback_set_led_state)

        self.set_dxl_leds_client = node.create_client(SetLeds,namespace + '/niryo_one/set_dxl_leds')

        # Subscribe to hotspot and hardware status. Those values will override standard states
        self.hardware_status_subscriber = node.create_subscription(HardwareStatus,namespace+'/niryo_one/hardware_status', self.callback_hardware_status,10)

        self.node.get_logger().info('LED manager has been started.')

    def set_dxl_leds(self,color):
        leds = [0, 0, 0, 8]  # gripper LED will not be used
        if color == LED_RED:
            leds = [1, 1, 1, 8]
        elif color == LED_GREEN:
            leds = [2, 2, 2, 8]
        elif color == LED_BLUE:
            leds = [4, 4, 4, 8]
        # 4 is yellow, no yellow
        elif color == LED_BLUE_GREEN:
            leds = [6, 6, 6, 8]
        elif color == LED_PURPLE:
            leds = [5, 5, 5, 8]
        elif color == LED_WHITE:
            leds = [7, 7, 7, 8]

        try:
            self.set_dxl_leds_client.wait_for_service(timeout_sec=1.0)
        except:
            self.node.get_logger().warn("Niryo ROS control LED service is not up!")
        try:
            self.req = SetLeds.Request()
            self.req.values = leds
            self.future = self.set_dxl_leds_client.call_async(self.req)
        except:
            self.node.get_logger().warn("Could not call /niryo_one/set_dxl_leds service")

    def set_led(self, color, dxl_leds=False):
        r = GPIO.LOW
        g = GPIO.LOW
        b = GPIO.LOW

        if color & 0b100:
            r = GPIO.HIGH
        if color & 0b010:
            g = GPIO.HIGH
        if color & 0b001:
            b = GPIO.HIGH

        GPIO.output(LED_GPIO_R, r)
        GPIO.output(LED_GPIO_G, g)
        GPIO.output(LED_GPIO_B, b)

        if dxl_leds:
            self.set_dxl_leds(color)

    def set_led_from_state(self, dxl_leds=False):
        if self.state == LedState.SHUTDOWN:
            self.set_led(LED_PURPLE, dxl_leds)
        elif self.state == LedState.HOTSPOT:
            self.set_led(LED_BLUE, dxl_leds)
        elif self.state == LedState.WAIT_HOTSPOT:
            self.set_led(LED_BLUE, dxl_leds)
        elif self.state == LedState.OK:
            self.set_led(LED_GREEN, dxl_leds)
        else:
            self.set_led(LED_OFF, dxl_leds)

    def callback_hardware_status(self, msg):
        if not msg.connection_up or msg.error_message != '':
            self.set_led(LED_RED, dxl_leds=True)  # blink red
            sleep(0.05)
            self.set_led_from_state(dxl_leds=True)

    def callback_set_led_state(self, req):
        state = req.value
        if state == LedState.SHUTDOWN:
            self.state = LedState.SHUTDOWN
            self.set_led_from_state(dxl_leds=True)
        elif state == LedState.WAIT_HOTSPOT:
            self.state = LedState.WAIT_HOTSPOT
            self.set_led_from_state(dxl_leds=True)
        else:
            return {'status': 400, 'message': 'Not yet implemented'}
        return {'status': 200, 'message': 'Set LED OK'}
