/*
    niryo_one_hardware_interface.h
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

#ifndef NIRYO_HARDWARE_INTERFACE_H
#define NIRYO_HARDWARE_INTERFACE_H

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <thread>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"


#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "niryo_one_driver/ros_interface.h"
#include "niryo_one_driver/fake_communication.h"
#include "niryo_one_driver/niryo_one_communication.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "niryo_one_msgs/srv/set_int.hpp"


namespace niryo_one_driver{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

std::shared_ptr<CommunicationBase> comm;
rclcpp::Node::SharedPtr node;
std::shared_ptr<RosInterface> ros_interface;
std::shared_ptr<RpiDiagnostics> rpi_diagnostics;

bool start_driver(){
  if(node != nullptr) return true;
  //Start Node
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);  
  options.automatically_declare_parameters_from_overrides(true);
  node = rclcpp::Node::make_shared("niryo_one_hardware_interface",options);

  RCLCPP_INFO(node->get_logger(), "Starting ...please wait...");

  //Get hardware version
  int hardware_version;
  node->get_parameter("hardware_version", hardware_version);

  //Check if Fake communicatioon
  bool fake_communication;
  node->get_parameter("fake_communication",fake_communication);
  
  //Return if wrong hardware version is set
  if (hardware_version != 1 && hardware_version != 2) {
      RCLCPP_ERROR(node->get_logger(),"Incorrect hardware version, should be 1 or 2");
      return false;
  }

  RCLCPP_INFO(node->get_logger(),"Starting NiryoOne communication");  
  if (fake_communication) {
      comm.reset(new FakeCommunication(hardware_version,node));
  }
  else {
      comm.reset(new NiryoOneCommunication(hardware_version,node));
  }
    
  //Init communication and check for errors
  int init_result = comm->init();
  if (init_result != 0) {
      return false;
  }
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1)));
  RCLCPP_INFO(rclcpp::get_logger("niryo_one_communication"),"NiryoOne communication has been successfully started");

  //Start communicaton control loop
  RCLCPP_INFO(node->get_logger(),"Start communication control loop");
  comm->manageHardwareConnection();
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));

  //Start Hardware interface Thread
  new std::thread([&](){
    //Start Raspberry PI Diagnostics
    RCLCPP_INFO(node->get_logger(),"Start Rpi Diagnostics...");
    rpi_diagnostics.reset(new RpiDiagnostics(node));

    //Get Learning mode on startup parameter
    bool learning_mode_activated_on_startup = true;
    //node->get_parameter("learning_mode_activated_on_startup",learning_mode_activated_on_startup);

    // activate learning mode 
    comm->activateLearningMode(learning_mode_activated_on_startup);

    RCLCPP_INFO(node->get_logger(),"Spinning Node");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(),"Shutdown Node");
    rclcpp::shutdown();
  });

  return true;
}

class NiryoOneHardwareInterface:  public hardware_interface::SystemInterface {

    public:
        
        CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) final;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        hardware_interface::return_type read() final;
        hardware_interface::return_type write() final;

        void ResetControllers();
        
    private:
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr service;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_controller_subscriber;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr trajectory_result_subscriber;

        void callbackTrajectoryGoal(const std_msgs::msg::Empty::SharedPtr msg);

        void callbackTrajectoryResult(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

        double cmd[6] = { 0, 0.64, -1.38, 0, 0, 0};
        double pos[6] = { 0, 0.64, -1.38, 0, 0, 0};
        double vel[6] = {0};
        double eff[6] = {0};

};
class NiryoOneActuatorInterface:  public hardware_interface::ActuatorInterface {

    public:
        
        CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) final;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        hardware_interface::return_type read() final;
        hardware_interface::return_type write() final;
        
    private:        
        double cmd = 0;
        double pos = 0;
        double eff = 0;

};
}
#endif
