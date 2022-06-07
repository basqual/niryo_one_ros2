/*
    ros_interface.cpp
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

#include "niryo_one_driver/ros_interface.h"

RosInterface::RosInterface(CommunicationBase* niryo_one_comm, RpiDiagnostics* rpi_diagnostics,
        std::function<void()> ResetControllers,rclcpp::Node::SharedPtr node)
{
    this->node = node;
    comm = niryo_one_comm;

    //Get Learning mode on startup parameter
    bool learning_mode_activated_on_startup = true;
    node->get_parameter("learning_mode_activated_on_startup",learning_mode_activated_on_startup);

    //Get hardware version
    int hardware_version;
    node->get_parameter("hardware_version", hardware_version);


    this->rpi_diagnostics = rpi_diagnostics;
    this->learning_mode_on = learning_mode_activated_on_startup;
    this->ResetControllers = ResetControllers;
    this->hardware_version = hardware_version;
    last_connection_up_flag = true;

    test_motor.reset(new NiryoOneTestMotor(node));
    
    node->get_parameter("image_version", rpi_image_version);
    node->get_parameter("ros_version", ros_niryo_one_version);
   
    // trim string
    rpi_image_version.erase(rpi_image_version.find_last_not_of(" \n\r\t")+1);
    ros_niryo_one_version.erase(ros_niryo_one_version.find_last_not_of(" \n\r\t")+1);
    
    RCLCPP_INFO(node->get_logger(),"Ros interface started.");

    startServiceServers();
    RCLCPP_INFO(node->get_logger(),"Ros Services started.");
    startPublishers();
    RCLCPP_INFO(node->get_logger(),"Ros Publishers started.");

    // this flag is used to know if learning mode can be deactivated
    calibration_needed = 0;
}

void RosInterface::callbackTestMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res) 
{    
    if (motor_test_status==1)
    {
        test_motor->stopTest();
        learning_mode_on = true;
        comm->activateLearningMode(learning_mode_on);
        return;
    }
    
    motor_test_status = 1;
    if (calibration_needed)
    {
        learning_mode_on = false;
        comm->activateLearningMode(learning_mode_on);
        
        int calibration_mode = 1; 
        std::string result_message = "";
        int result = comm->allowMotorsCalibrationToStart(calibration_mode, result_message);

        //ros::Duration(1).sleep();
        sleep_for(1);
        while (calibration_in_progress) { sleep_for(0.05);}

        learning_mode_on = true;
        //ros::Duration(1).sleep();
        sleep_for(1);
    }
    
    learning_mode_on = false;
    comm->activateLearningMode(learning_mode_on);

    bool status = test_motor->runTest(req->value);

    learning_mode_on = true;
    comm->activateLearningMode(learning_mode_on);

    if (status)
    {
        motor_test_status = 0;
        res->status = 200;
        res->message = "Success";
        RCLCPP_INFO(node->get_logger(),"Motor debug has ended with success");
    }
    else
    {
        motor_test_status = -1;
        res->status = 400;
        res->message = "Fail";
        RCLCPP_ERROR(node->get_logger(),"Motor debug has ended with failure");
    }
}


void RosInterface::callbackCalibrateMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req,niryo_one_msgs::srv::SetInt::Response::SharedPtr res) 
{
    int calibration_mode = req->value; 
    std::string result_message = "";
    int result = comm->allowMotorsCalibrationToStart(calibration_mode, result_message);

    res->status = result;
    res->message = result_message;

    // special case here 
    // we set flag learning_mode_on, but we don't activate from here
    // learning_mode should be activated in comm, AFTER motors have been calibrated
    // --> this fixes an issue where motors will jump back to a previous cmd after being calibrated
    learning_mode_on = true;
    comm->activateLearningMode(learning_mode_on);
}

void RosInterface::callbackRequestNewCalibration(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res)
{
    // 1. Activate learning mode
    learning_mode_on = true;
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::msg::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher->publish(msg);

    // 2. Set calibration flag (user will have to validate for calibration to start)
    comm->requestNewCalibration();

    res->status = 200;
    res->message = "New calibration request has been made, you will be requested to confirm it.";
}

/*
 * Deactivating learning mode (= activating motors) is possible only if motors are calibrated
 * Activating learning mode is also possible when waiting for calibration
 */
void RosInterface::callbackActivateLearningMode(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res)
{
    using namespace std::chrono_literals;
    if (comm->isCalibrationInProgress()) {
        res->status = 400;
        res->message = "You can't activate/deactivate learning mode during motors calibration";
        return;
    }

    if (calibration_needed == 1 || !comm->isConnectionOk()) { // if can or dxl is disconnected, only allow to activate learning mode
        learning_mode_on = true;
    }
    else {
        learning_mode_on = req->value; 
    }
    
    // reset controller if learning mode -> OFF
    // we want motors to start where they physically are, not from the last command
    if (!learning_mode_on) {
        ResetControllers();
        sleep_for(0.05);
        //ros::Duration(0.05).sleep();
    }
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::msg::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher->publish(msg);
   
    res->status = 200;
    res->message = (learning_mode_on) ? "Activating learning mode" : "Deactivating learning mode";
}

void RosInterface::callbackActivateLeds(const niryo_one_msgs::srv::SetLeds::Request::SharedPtr req, niryo_one_msgs::srv::SetLeds::Response::SharedPtr res)
{
    std::vector<int> leds = req->values;
    std::string message = "";
    bool result = comm->setLeds(leds, message);

    res->status = (result) ? 200 : 400;
    res->message = message;
}

void RosInterface::callbackPingAndSetDxlTool(const niryo_one_msgs::srv::PingDxlTool::Request::SharedPtr req, niryo_one_msgs::srv::PingDxlTool::Response::SharedPtr res)
{
    res->state = comm->pingAndSetDxlTool(req->id, req->name);
}

void RosInterface::callbackPingAndSetConveyor(const niryo_one_msgs::srv::SetConveyor::Request::SharedPtr req, niryo_one_msgs::srv::SetConveyor::Response::SharedPtr res) {
    std::string message = "";
    res->status = comm->pingAndSetConveyor(req->id, req->activate, message);
    res->message = message; 
}
void RosInterface::callbackControlConveyor(const niryo_one_msgs::srv::ControlConveyor::Request::SharedPtr req, niryo_one_msgs::srv::ControlConveyor::Response::SharedPtr res){
    std::string message = "";
    res->status = comm->moveConveyor(req->id, req->control_on, req->speed, req->direction, message);
    res->message = message; 
}
void  RosInterface::callbackUpdateIdConveyor(const niryo_one_msgs::srv::UpdateConveyorId::Request::SharedPtr req, niryo_one_msgs::srv::UpdateConveyorId::Response::SharedPtr res){
    std::string message = "";
    res->status = comm->updateIdConveyor(req->old_id, req->new_id, message);
    res->message = message;
}

void RosInterface::callbackOpenGripper(const niryo_one_msgs::srv::OpenGripper::Request::SharedPtr req, niryo_one_msgs::srv::OpenGripper::Response::SharedPtr res)
{
    res->state = comm->openGripper(req->id, req->open_position, req->open_speed, req->open_hold_torque);
}

void RosInterface::callbackCloseGripper(const niryo_one_msgs::srv::CloseGripper::Request::SharedPtr req, niryo_one_msgs::srv::CloseGripper::Response::SharedPtr res)
{
    res->state = comm->closeGripper(req->id, req->close_position, req->close_speed, req->close_hold_torque, req->close_max_torque);
}

void RosInterface::callbackPullAirVacuumPump(const niryo_one_msgs::srv::PullAirVacuumPump::Request::SharedPtr req, niryo_one_msgs::srv::PullAirVacuumPump::Response::SharedPtr res)
{
    res->state = comm->pullAirVacuumPump(req->id, req->pull_air_position, req->pull_air_hold_torque);
}

void RosInterface::callbackPushAirVacuumPump(const niryo_one_msgs::srv::PushAirVacuumPump::Request::SharedPtr req, niryo_one_msgs::srv::PushAirVacuumPump::Response::SharedPtr res)
{
    res->state = comm->pushAirVacuumPump(req->id, req->push_air_position);
}

void RosInterface::callbackSendCustomDxlValue(const niryo_one_msgs::srv::SendCustomDxlValue::Request::SharedPtr req,
        niryo_one_msgs::srv::SendCustomDxlValue::Response::SharedPtr res)
{
    // pre-check motor type
    if (req->motor_type != 1 && req->motor_type != 2) {
        res->status = 400;
        res->message = "Invalid motor type: should be 1 (XL-320) or 2 (XL-430)";
        return;
    }

    comm->addCustomDxlCommand(req->motor_type, req->id, req->value, req->reg_address, req->byte_number);

    res->status = 200;
    res->message = "OK";
}

void RosInterface::callbackRebootMotors(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res)
{
    comm->rebootMotors();
    res->status = 200;
    res->message = "OK";
}

void RosInterface::callbackSetGripperTorque(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res)
{
    comm->setGripperTorque(req->value);
    res->status = 200;
    res->message = "OK";
}

void RosInterface::callbackSetGripperVelocity(const niryo_one_msgs::srv::SetInt::Request::SharedPtr req, niryo_one_msgs::srv::SetInt::Response::SharedPtr res)
{
    comm->setGripperVelocity(req->value);
    res->status = 200;
    res->message = "OK";
}

void RosInterface::startServiceServers()
{
    calibrate_motors_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/calibrate_motors", std::bind(&RosInterface::callbackCalibrateMotors, this, std::placeholders::_1, std::placeholders::_2));
    request_new_calibration_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/request_new_calibration",std::bind(&RosInterface::callbackRequestNewCalibration, this, std::placeholders::_1, std::placeholders::_2) );

    test_motors_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/test_motors", std::bind(&RosInterface::callbackTestMotors, this, std::placeholders::_1, std::placeholders::_2));

    activate_learning_mode_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/activate_learning_mode",std::bind(&RosInterface::callbackActivateLearningMode, this, std::placeholders::_1, std::placeholders::_2) );
    activate_leds_server = node->create_service<niryo_one_msgs::srv::SetLeds>("niryo_one/set_dxl_leds",std::bind(&RosInterface::callbackActivateLeds, this, std::placeholders::_1, std::placeholders::_2) );

    ping_and_set_dxl_tool_server = node->create_service<niryo_one_msgs::srv::PingDxlTool>("niryo_one/tools/ping_and_set_dxl_tool",std::bind(&RosInterface::callbackPingAndSetDxlTool, this, std::placeholders::_1, std::placeholders::_2) );

    // steppers service test
    ping_and_set_stepper_server = node->create_service<niryo_one_msgs::srv::SetConveyor>("niryo_one/kits/ping_and_set_conveyor",std::bind(&RosInterface::callbackPingAndSetConveyor, this, std::placeholders::_1, std::placeholders::_2) );
    control_conveyor_server = node->create_service<niryo_one_msgs::srv::ControlConveyor>("niryo_one/kits/control_conveyor",std::bind(&RosInterface::callbackControlConveyor, this, std::placeholders::_1, std::placeholders::_2) );
    update_conveyor_id_server = node->create_service<niryo_one_msgs::srv::UpdateConveyorId>("niryo_one/kits/update_conveyor_id",std::bind(&RosInterface::callbackUpdateIdConveyor, this, std::placeholders::_1, std::placeholders::_2) );

    open_gripper_server = node->create_service<niryo_one_msgs::srv::OpenGripper>("niryo_one/tools/open_gripper",std::bind(&RosInterface::callbackOpenGripper, this, std::placeholders::_1, std::placeholders::_2) );
    close_gripper_server = node->create_service<niryo_one_msgs::srv::CloseGripper>("niryo_one/tools/close_gripper",std::bind(&RosInterface::callbackCloseGripper, this, std::placeholders::_1, std::placeholders::_2) );
    pull_air_vacuum_pump_server = node->create_service<niryo_one_msgs::srv::PullAirVacuumPump>("niryo_one/tools/pull_air_vacuum_pump",std::bind(&RosInterface::callbackPullAirVacuumPump, this, std::placeholders::_1, std::placeholders::_2) );
    push_air_vacuum_pump_server = node->create_service<niryo_one_msgs::srv::PushAirVacuumPump>("niryo_one/tools/push_air_vacuum_pump",std::bind(&RosInterface::callbackPushAirVacuumPump, this, std::placeholders::_1, std::placeholders::_2) );

    send_custom_dxl_value_server = node->create_service<niryo_one_msgs::srv::SendCustomDxlValue>("niryo_one/send_custom_dxl_value",std::bind(&RosInterface::callbackSendCustomDxlValue, this, std::placeholders::_1, std::placeholders::_2) );
    reboot_motors_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/reboot_motors",std::bind(&RosInterface::callbackRebootMotors, this, std::placeholders::_1, std::placeholders::_2) );

    set_gripper_torque_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/gripper/torque",std::bind(&RosInterface::callbackSetGripperTorque, this, std::placeholders::_1, std::placeholders::_2) );
    set_gripper_velocity_server = node->create_service<niryo_one_msgs::srv::SetInt>("niryo_one/gripper/velocity",std::bind(&RosInterface::callbackSetGripperVelocity, this, std::placeholders::_1, std::placeholders::_2) );

}

void RosInterface::publishHardwareStatus()
{
    double publish_hw_status_frequency;
    node->get_parameter("publish_hw_status_frequency", publish_hw_status_frequency);
    rclcpp::Rate publish_hardware_status_rate(publish_hw_status_frequency);
    while (rclcpp::ok()) {
        rclcpp::Time time_now = node->now();

        bool connection_up = false;
        
        std::string error_message;
        std::vector<std::string> motor_names;
        std::vector<std::string> motor_types;
        std::vector<int32_t> temperatures;
        std::vector<double> voltages;
        std::vector<int32_t> hw_errors;

        comm->getHardwareStatus(&connection_up, error_message, &calibration_needed, 
                &calibration_in_progress, motor_names, motor_types, temperatures, voltages, hw_errors);
        if (connection_up && !last_connection_up_flag) {
            learning_mode_on = true;
            comm->activateLearningMode(learning_mode_on);
            
            // publish one time
            std_msgs::msg::Bool msg;
            msg.data = learning_mode_on;
            learning_mode_publisher->publish(msg);
        }
        last_connection_up_flag = connection_up;
        niryo_one_msgs::msg::HardwareStatus msg;
        msg.header.stamp = node->now();
        msg.rpi_temperature = rpi_diagnostics->getRpiCpuTemperature();
        msg.hardware_version = hardware_version;
        msg.connection_up = connection_up;
        if (motor_test_status<0)
        {
            error_message += " motor test error";
        }
        msg.error_message = error_message;
        msg.calibration_needed = calibration_needed;
        msg.calibration_in_progress = calibration_in_progress;
        msg.motor_names = motor_names;
        msg.motor_types = motor_types;
        msg.temperatures = temperatures;
        msg.voltages = voltages;
        msg.hardware_errors = hw_errors;
        hardware_status_publisher->publish(msg);
        publish_hardware_status_rate.sleep();
        
    }
}

void RosInterface::publishSoftwareVersion()
{
    
    double publish_software_version_frequency;
    node->get_parameter("publish_software_version_frequency", publish_software_version_frequency);
    rclcpp::Rate publish_software_version_rate(publish_software_version_frequency);

    while (rclcpp::ok()) {
        std::vector<std::string> motor_names;
        std::vector<std::string> firmware_versions;
        comm->getFirmwareVersions(motor_names, firmware_versions);

        niryo_one_msgs::msg::SoftwareVersion msg;
        msg.motor_names = motor_names;
        msg.stepper_firmware_versions = firmware_versions;
        msg.rpi_image_version = rpi_image_version;
        msg.ros_niryo_one_version = ros_niryo_one_version;
       
        software_version_publisher->publish(msg);
        publish_software_version_rate.sleep();
        
    }
    
}

void RosInterface::publishLearningMode()
{
    double publish_learning_mode_frequency;
    node->get_parameter("publish_learning_mode_frequency", publish_learning_mode_frequency);
    rclcpp::Rate publish_learning_mode_rate(publish_learning_mode_frequency);

    while (rclcpp::ok()) {
        std_msgs::msg::Bool msg;
        msg.data = learning_mode_on;
        learning_mode_publisher->publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

void RosInterface::publishConveyor1Feedback()
{   
    double publish_conveyor_feedback_frequency = 2.0;
    rclcpp::Rate publish_conveyor_feedback_rate(publish_conveyor_feedback_frequency);

    while (rclcpp::ok()) {
        niryo_one_msgs::msg::ConveyorFeedback msg;
        bool connection_state;
        bool running;
        int16_t speed;
        int8_t direction; 
        comm->getConveyorFeedBack (6, &connection_state, &running, &speed, &direction); 
        msg.conveyor_id  = 6;
        msg.connection_state = connection_state; 
        msg.running = running;
        msg.speed = speed; 
        msg.direction = direction;  

        conveyor_1_feedback_publisher->publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}

void RosInterface::publishConveyor2Feedback()
{   
    double publish_conveyor_feedback_frequency = 2.0;
    rclcpp::Rate publish_conveyor_feedback_rate(publish_conveyor_feedback_frequency);

    while (rclcpp::ok()) {
        niryo_one_msgs::msg::ConveyorFeedback msg;
        bool connection_state;
        bool running;
        int16_t speed;
        int8_t direction; 
        comm->getConveyorFeedBack (7, &connection_state, &running, &speed, &direction); 
        msg.conveyor_id  = 7;
        msg.connection_state = connection_state; 
        msg.running = running;
        msg.speed = speed; 
        msg.direction = direction;  

        conveyor_2_feedback_publisher->publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}

void RosInterface::startPublishers()
{
    
    hardware_status_publisher = node->create_publisher<niryo_one_msgs::msg::HardwareStatus>("niryo_one/hardware_status", 10);
    publish_hardware_status_thread.reset(new std::thread(std::bind(&RosInterface::publishHardwareStatus, this))); 
    
    software_version_publisher = node->create_publisher<niryo_one_msgs::msg::SoftwareVersion>("niryo_one/software_version", 10);
    publish_software_version_thread.reset(new std::thread(std::bind(&RosInterface::publishSoftwareVersion, this)));

    learning_mode_publisher = node->create_publisher<std_msgs::msg::Bool>("niryo_one/learning_mode", 10);
    publish_learning_mode_thread.reset(new std::thread(std::bind(&RosInterface::publishLearningMode, this)));
    
    conveyor_1_feedback_publisher = node->create_publisher<niryo_one_msgs::msg::ConveyorFeedback>("niryo_one/kits/conveyor_1_feedback", 10);
    publish_conveyor_1_feedback_thread.reset(new std::thread(std::bind(&RosInterface::publishConveyor1Feedback, this)));

    conveyor_2_feedback_publisher = node->create_publisher<niryo_one_msgs::msg::ConveyorFeedback>("niryo_one/kits/conveyor_2_feedback", 10);
    publish_conveyor_2_feedback_thread.reset(new std::thread(std::bind(&RosInterface::publishConveyor2Feedback, this)));  
    
}
