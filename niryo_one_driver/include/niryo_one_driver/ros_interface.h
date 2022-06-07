/*
    ros_interface.h
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

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "niryo_one_driver/communication_base.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "niryo_one_driver/test_motors.h"

#include "niryo_one_msgs/srv/set_int.hpp"
#include "niryo_one_msgs/srv/set_leds.hpp"

#include "niryo_one_msgs/srv/ping_dxl_tool.hpp"
#include "niryo_one_msgs/srv/open_gripper.hpp"
#include "niryo_one_msgs/srv/close_gripper.hpp"
#include "niryo_one_msgs/srv/pull_air_vacuum_pump.hpp"
#include "niryo_one_msgs/srv/push_air_vacuum_pump.hpp"

#include "niryo_one_msgs/srv/send_custom_dxl_value.hpp"
#include "niryo_one_msgs/srv/set_conveyor.hpp"
#include "niryo_one_msgs/srv/control_conveyor.hpp"
#include "niryo_one_msgs/srv/update_conveyor_id.hpp"

#include "niryo_one_msgs/msg/hardware_status.hpp"
#include "niryo_one_msgs/msg/software_version.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "niryo_one_msgs/msg/conveyor_feedback.hpp"

void sleep_for(double seconds);

class RosInterface {

    public:

        RosInterface(CommunicationBase* niryo_one_comm, RpiDiagnostics* rpi_diagnostics,
                std::function<void()> ResetControllers,rclcpp::Node::SharedPtr node);

        void startServiceServers();
        void startPublishers();
        void startSubscribers();

    private:

        CommunicationBase* comm;
        RpiDiagnostics* rpi_diagnostics;
        //ros::NodeHandle nh_;
        rclcpp::Node::SharedPtr node;

        std::shared_ptr<NiryoOneTestMotor> test_motor; 

        std::function<void()> ResetControllers;
        int hardware_version;
        bool learning_mode_on;
        int calibration_needed;
        bool calibration_in_progress;
        bool last_connection_up_flag;
        int motor_test_status;

        std::string rpi_image_version;
        std::string ros_niryo_one_version;

        // publishers

        rclcpp::Publisher<niryo_one_msgs::msg::HardwareStatus>::SharedPtr hardware_status_publisher;
        std::shared_ptr<std::thread> publish_hardware_status_thread;

        rclcpp::Publisher<niryo_one_msgs::msg::SoftwareVersion>::SharedPtr software_version_publisher;
        std::shared_ptr<std::thread> publish_software_version_thread;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr learning_mode_publisher;
        std::shared_ptr<std::thread> publish_learning_mode_thread;

        rclcpp::Publisher<niryo_one_msgs::msg::ConveyorFeedback>::SharedPtr conveyor_status_publisher;
        std::shared_ptr<std::thread> publish_conveyor_status_thread;
        
        rclcpp::Publisher<niryo_one_msgs::msg::ConveyorFeedback>::SharedPtr conveyor_1_feedback_publisher;
        std::shared_ptr<std::thread> publish_conveyor_1_feedback_thread;

        rclcpp::Publisher<niryo_one_msgs::msg::ConveyorFeedback>::SharedPtr conveyor_2_feedback_publisher;
        std::shared_ptr<std::thread> publish_conveyor_2_feedback_thread;

        // publish methods

        void publishHardwareStatus();
        void publishSoftwareVersion();
        void publishLearningMode(); 
        void publishConveyor1Feedback();
        void publishConveyor2Feedback(); 

        // services

        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr calibrate_motors_server;
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr request_new_calibration_server;

        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr test_motors_server;

        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr activate_learning_mode_server;
        rclcpp::Service<niryo_one_msgs::srv::SetLeds>::SharedPtr activate_leds_server;

        rclcpp::Service<niryo_one_msgs::srv::PingDxlTool>::SharedPtr ping_and_set_dxl_tool_server;

        rclcpp::Service<niryo_one_msgs::srv::OpenGripper>::SharedPtr open_gripper_server;
        rclcpp::Service<niryo_one_msgs::srv::CloseGripper>::SharedPtr close_gripper_server;
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr set_gripper_velocity_server;
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr set_gripper_torque_server;

        rclcpp::Service<niryo_one_msgs::srv::PullAirVacuumPump>::SharedPtr pull_air_vacuum_pump_server;
        rclcpp::Service<niryo_one_msgs::srv::PushAirVacuumPump>::SharedPtr push_air_vacuum_pump_server;

        rclcpp::Service<niryo_one_msgs::srv::SendCustomDxlValue>::SharedPtr send_custom_dxl_value_server;
        rclcpp::Service<niryo_one_msgs::srv::SetInt>::SharedPtr reboot_motors_server;

        // Conveyor services
        rclcpp::Service<niryo_one_msgs::srv::SetConveyor>::SharedPtr ping_and_set_stepper_server;
        rclcpp::Service<niryo_one_msgs::srv::ControlConveyor>::SharedPtr control_conveyor_server;
        rclcpp::Service<niryo_one_msgs::srv::UpdateConveyorId>::SharedPtr update_conveyor_id_server;

        // callbacks
        void callbackTestMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);

        void callbackCalibrateMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);
        void callbackRequestNewCalibration(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);

        void callbackActivateLearningMode(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);
        void callbackActivateLeds(const niryo_one_msgs::srv::SetLeds::Request::SharedPtr req, niryo_one_msgs::srv::SetLeds::Response::SharedPtr res);

        void callbackPingAndSetDxlTool(const niryo_one_msgs::srv::PingDxlTool::Request::SharedPtr req, niryo_one_msgs::srv::PingDxlTool::Response::SharedPtr res);

        void callbackPingAndSetConveyor(const niryo_one_msgs::srv::SetConveyor::Request::SharedPtr req, niryo_one_msgs::srv::SetConveyor::Response::SharedPtr res);
        void callbackControlConveyor(const niryo_one_msgs::srv::ControlConveyor::Request::SharedPtr req, niryo_one_msgs::srv::ControlConveyor::Response::SharedPtr res);
        void callbackUpdateIdConveyor(const niryo_one_msgs::srv::UpdateConveyorId::Request::SharedPtr req, niryo_one_msgs::srv::UpdateConveyorId::Response::SharedPtr res);

        void callbackOpenGripper(const niryo_one_msgs::srv::OpenGripper::Request::SharedPtr req, niryo_one_msgs::srv::OpenGripper::Response::SharedPtr res);
        void callbackCloseGripper(const niryo_one_msgs::srv::CloseGripper::Request::SharedPtr req, niryo_one_msgs::srv::CloseGripper::Response::SharedPtr res);

        void callbackPullAirVacuumPump(const niryo_one_msgs::srv::PullAirVacuumPump::Request::SharedPtr req, niryo_one_msgs::srv::PullAirVacuumPump::Response::SharedPtr res);
        void callbackPushAirVacuumPump(niryo_one_msgs::srv::PushAirVacuumPump::Request::SharedPtr req, niryo_one_msgs::srv::PushAirVacuumPump::Response::SharedPtr res);

        void callbackSendCustomDxlValue(const niryo_one_msgs::srv::SendCustomDxlValue::Request::SharedPtr req, 
                niryo_one_msgs::srv::SendCustomDxlValue::Response::SharedPtr res);

        void callbackRebootMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);

        void callbackSetGripperTorque(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);
        void callbackSetGripperVelocity(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res);
};

#endif
