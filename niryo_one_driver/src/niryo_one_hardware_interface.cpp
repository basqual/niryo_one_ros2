/*
    niryo_one_hardware_interface.cpp
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "niryo_one_driver/niryo_one_hardware_interface.h"

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(niryo_one_driver::NiryoOneHardwareInterface, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(niryo_one_driver::NiryoOneActuatorInterface, hardware_interface::ActuatorInterface)

using namespace niryo_one_driver;

CallbackReturn NiryoOneHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  info_ = system_info;

  if(start_driver()) {

    //Get Node Namespace
    std::string ns = node->get_namespace();
    ns = ns=="/"?"":ns;  
    //Start Subscriber to get Stepper reset message
    reset_controller_subscriber = node->create_subscription<std_msgs::msg::Empty>(ns+"/niryo_one/steppers_reset_controller", 10,
                std::bind(&NiryoOneHardwareInterface::callbackTrajectoryGoal,this, std::placeholders::_1));

    trajectory_result_subscriber = node->create_subscription<action_msgs::msg::GoalStatusArray>(ns+"/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/_action/status",10,
      std::bind(&NiryoOneHardwareInterface::callbackTrajectoryResult,this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),"Starting ROS interface...");
    ros_interface.reset(new RosInterface(comm.get(), rpi_diagnostics.get(),std::bind(&NiryoOneHardwareInterface::ResetControllers,this), node));

    return CallbackReturn::SUCCESS;
  }
  else return CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> NiryoOneHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < 6; i++)
  {
    state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_POSITION, &pos[i]));
    state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_VELOCITY, &vel[i]));
    state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_EFFORT, &eff[i]));   
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NiryoOneHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < 6; i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("hardware_interface"),"Joint name: %s",info_.joints[i].name.c_str());
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name,hardware_interface::HW_IF_POSITION, &cmd[i])); 
  }
  return command_interfaces;
}
hardware_interface::return_type NiryoOneHardwareInterface::write()
{
    comm->sendPositionToRobot(cmd);
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type NiryoOneHardwareInterface::read()
{
  double pos_to_read[6] = {0.0};

  comm->getCurrentPosition(pos_to_read);

  for (uint i = 0; i < 6; i++)
  {
      pos[i] = pos_to_read[i];
  }

  return hardware_interface::return_type::OK;
}

void NiryoOneHardwareInterface::ResetControllers(){
  for(int i = 0;i<6;i++)cmd[i] = pos[i];
  comm->synchronizeMotors(true);
  RCLCPP_INFO(node->get_logger(),"Setting Controller to current position");
}

    
void NiryoOneHardwareInterface::callbackTrajectoryGoal(const std_msgs::msg::Empty::SharedPtr msg)
{
    NiryoOneHardwareInterface::ResetControllers();
    comm->synchronizeMotors(true);
}

void NiryoOneHardwareInterface::callbackTrajectoryResult(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if(msg->status_list[0].status == 4){
    comm->synchronizeMotors(false);
  }
}    

CallbackReturn NiryoOneActuatorInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::ActuatorInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  info_ = system_info;

  if(start_driver()) return CallbackReturn::SUCCESS;
  else return CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> NiryoOneActuatorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
          hardware_interface::StateInterface(info_.joints[0].name,hardware_interface::HW_IF_POSITION, &pos));
  state_interfaces.emplace_back(
          hardware_interface::StateInterface(info_.joints[0].name,hardware_interface::HW_IF_VELOCITY, &vel));
  state_interfaces.emplace_back(
          hardware_interface::StateInterface(info_.joints[0].name,hardware_interface::HW_IF_EFFORT, &eff));   
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NiryoOneActuatorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(rclcpp::get_logger("actuator_interface"),"Joint name: %s",info_.joints[0].name.c_str());
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name,hardware_interface::HW_IF_POSITION, &cmd)); 
  return command_interfaces;
}
hardware_interface::return_type NiryoOneActuatorInterface::write()
{
    comm->sendGripperPositionToRobot(cmd);
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type NiryoOneActuatorInterface::read()
{
  double pos_to_read = 0.0;
  
  comm->getCurrentGripperPosition(pos_to_read);

  pos = pos_to_read;
  
  return hardware_interface::return_type::OK;
}