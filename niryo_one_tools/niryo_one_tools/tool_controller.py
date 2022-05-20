#!/usr/bin/env python

# tool_controller.py
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
from rclpy.action import ActionServer
from rclpy.node import Node

from niryo_one_msgs.srv import SetInt

from niryo_one_msgs.action import Tool

from rclpy.duration import Duration


from std_msgs.msg import Int32

from niryo_one_tools.tools import *
from niryo_one_tools.tool_ros_command_interface import ToolRosCommandInterface


class ToolController:
    def __init__(self, ros_command_interface,node):

        self.ros_command_interface = ros_command_interface

        self.node = node

        self.server = ActionServer(node, Tool, 'niryo_one/tool_action',execute_callback=self.execute_callback,goal_callback=self.tool_on_goal)

        self.change_tool_server = self.node.create_service(SetInt,'niryo_one/change_tool', self.callback_change_tool)

        self.current_tool_id_publisher = self.node.create_publisher(Int32,'/niryo_one/current_tool_id', 1)

        node.create_timer(1 / 1.0, self.publish_current_tool_id)

        self.current_tool = None
        self.available_tools = None
        self.command_list = None
        self.create_tools()

    def execute_callback(self,goal_handle):
        pass

    def create_tools(self):

        # Get params from rosparams
        tool_config_list = self.node.get_parameter("tool_list").value
        tool_command_list = self.node.get_parameter("command_list")

        self.available_tools = []
        self.command_list = tool_command_list

        for toolStr in tool_config_list:
            self.node.get_logger().info(toolStr)
            type = self.node.get_parameter("tools."+toolStr+".type").value
            id = self.node.get_parameter("tools."+toolStr+".id").value
            available_commands = self.node.get_parameter("tools."+toolStr+".available_commands").value

            self.node.get_parameter("tools."+toolStr+".specs.").value

            new_tool = None

            if type== 'gripper':
                new_tool = Gripper(id, toolStr, self.ros_command_interface, self.getSpec(toolStr,'open_position'),
                                   self.getSpec(toolStr,'open_hold_torque') ,self.getSpec(toolStr,'close_position'),
                                   self.getSpec(toolStr,'close_hold_torque'), self.getSpec(toolStr,'close_max_torque'))
            elif type == 'electromagnet':
                new_tool = Electromagnet(id, toolStr, self.ros_command_interface)
            elif type == 'vacuum_pump':
                new_tool = VacuumPump(id, toolStr, self.ros_command_interface,
                                      self.getSpec(toolStr,'pull_air_position'),
                                      self.getSpec(toolStr,'pull_air_hold_torque'), self.getSpec(toolStr,'push_air_position'))
            else:
                self.node.get_logger().warn("ERROR : TYPE NOT RECOGNIZED from tool config list : " + str(tool['type']))
                continue

            self.available_tools.append(new_tool)

            cmds = []

            for cmd in available_commands:
                try:
                    cmds.append(self.node.get_parameter("command_list."+cmd).value)
                except Exception as e:
                    self.node.get_logger().warn(str(cmd) + " is not a recognized command")
                    continue

            new_tool.set_available_commands(cmds)

        self.node.get_logger().info("Detected tools : (id + available commands)")
        for tool in self.available_tools:
            self.node.get_logger().info(str(tool.get_id()) + ' : ' + str(tool.get_available_commands()))
    
    def getSpec(self,toolString,spec):
        return self.node.get_parameter("tools."+toolString+".id."+spec).value

    def tool_on_goal(self, goal):
        cmd = goal.cmd
        # rospy.loginfo("Tool controller - received goal : " + str(goal))

        if self.current_tool is None:
            self.server.set_aborted(self.create_action_result(400, "No tool selected"))
            return

        # 1. Check tool id
        if cmd.tool_id != self.current_tool.get_id():
            self.server.set_aborted(self.create_action_result(400,
                                                              "Wrong tool selected - ID does not match : " + str(
                                                                  self.current_tool.get_id())))
            return

        # 2. Check if current tool is active
        if self.current_tool.is_active:
            self.server.set_aborted(self.create_action_result(400, "Tool still active, retry later"))
            return

        # 3. Check cmd_type (if exists, and is available for selected tool)
        if cmd.cmd_type not in self.current_tool.get_available_commands():
            self.server.set_aborted(self.create_action_result(400,
                                                              "Command type is not available for current tool (not in " + str(
                                                                  self.current_tool.get_available_commands()) + ")"))
            return

        # 3.1 Validate params
        try:
            self.current_tool.validate_command(cmd)
        except ToolValidationException as e:
            self.server.set_aborted(self.create_action_result(400, str(e)))
            return

        # 4. Execute cmd -> retrieve cmd name in command list and execute on current tool
        self.current_tool.set_as_active()
        success = False
        message = "Error"

        function_name = self.command_list.keys()[self.command_list.values().index(cmd.cmd_type)]
        success, message = getattr(self.current_tool, function_name)(cmd)  # Execute function from name

        self.current_tool.set_as_non_active()

        # 5. Return success or error
        if success:
            self.server.set_succeeded(
                self.create_action_result(200, "Tool action successfully finished"))
        else:
            self.node.get_logger().info("Tool controller - error : " + str(message))
            self.server.set_aborted(self.create_action_result(400, message))

    def callback_change_tool(self, req):
        new_tool_id = req.value

        # Check if action == detach tool (id == 0)
        if new_tool_id == 0:
            self.ros_command_interface.ping_dxl_tool(0, 'No Dxl Tool')
            self.current_tool = None
            self.publish_current_tool_id(None)
            return self.create_response(200, "Tool has been detached")

        # Check if tool id is null
        if new_tool_id is None:
            return self.create_response(400, "No tool ID provided")

        if self.current_tool is not None:
            if self.current_tool.get_id() == new_tool_id:
                return self.create_response(200, "This tool has already been selected")
            if self.current_tool.is_active:
                return self.create_response(400, "Current tool is still active, please retry later")

        # look for new tool in available tools array
        for tool in self.available_tools:
            if new_tool_id == tool.get_id():
                if tool.get_type() == "gripper" or tool.get_type() == "vacuum_pump":  # extra check for dynamixel tools
                    if not tool.is_connected():
                        return self.create_response(400, "Correct ID but Dynamixel tool is not connected")
                else:
                    self.ros_command_interface.ping_dxl_tool(0,
                                                             'No Dxl Tool')  # remove dxl tool if another kind of tool is selected
                self.current_tool = tool
                self.publish_current_tool_id(None)
                return self.create_response(200, "New tool has been selected, id : " + str(new_tool_id))

        # no tool found in available tools
        return self.create_response(400, "This ID does not match any available tool ID")

    @staticmethod
    def create_action_result(status, message):
        result = ToolResult()
        result.status = status
        result.message = message
        return result

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def publish_current_tool_id(self):
        msg = Int32()
        if self.current_tool:
            msg.data = self.current_tool.id
        else:
            msg.data = 0
        self.current_tool_id_publisher.publish(msg)

    def start(self):
        self.node.get_logger().info('Action server started (ToolController)')

def main(args = None):
    rclpy.init(args=args)
    node = rclpy.create_node('niryo_one_tools',allow_undeclared_parameters=True,automatically_declare_parameters_from_overrides=True)
    node.get_logger().info('Created node')
    ros_command_interface = ToolRosCommandInterface(node)
    tc = ToolController(ros_command_interface,node)
    tc.start()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
