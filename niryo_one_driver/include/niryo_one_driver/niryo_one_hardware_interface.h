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

class NiryoOneHardwareInterface:  public hardware_interface::SystemInterface {

    public:
        
        CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) final;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        hardware_interface::return_type read() final;
        hardware_interface::return_type write() final;

        void ResetControllers();
        
    private:
        
        std::shared_ptr<CommunicationBase> comm;
        rclcpp::Node::SharedPtr node;
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr service;

        std::shared_ptr<RosInterface> ros_interface;
        std::shared_ptr<std::thread> hw_interface_thread;

        std::shared_ptr<RpiDiagnostics> rpi_diagnostics;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_controller_subscriber;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr trajectory_result_subscriber;

        void callbackTrajectoryGoal(const std_msgs::msg::Empty::SharedPtr msg);

        void callbackTrajectoryResult(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
        void NiryoOneHIThread();
        
        double cmd[6] = { 0, 0.64, -1.38, 0, 0, 0};
        double pos[6] = { 0, 0.64, -1.38, 0, 0, 0};
        double vel[6] = {0};
        double eff[6] = {0};

};
}
#endif
