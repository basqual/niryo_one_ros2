#include "niryo_one_driver/test_motors.h"

NiryoOneTestMotor::NiryoOneTestMotor(rclcpp::Node::SharedPtr node)
{
    this->node = node;
    getJointsLimits();

    std::string ns = node->get_namespace();
    ns = ns=="/"?"":ns;
    
    reset_stepper_publisher = node->create_publisher<std_msgs::msg::Empty>(ns+"/niryo_one/steppers_reset_controller", 1000);
    calibrate_motor_client = node->create_client<niryo_one_msgs::srv::SetInt>("/niryo_one/calibrate_motors");
    
    this->traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node,ns+"/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory");
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    joint_state_subscriber = node->create_subscription<sensor_msgs::msg::JointState>(ns+"/joint_states",qos,std::bind(&NiryoOneTestMotor::callbackJointSate, this, std::placeholders::_1)); //std::bind(&NiryoOneTestMotor::callbackJointSate, this,std::placeholders::_1));
    
    RCLCPP_INFO(node->get_logger(),"Test motors up");
}

void NiryoOneTestMotor::callbackJointSate(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    _current_joint_pose.resize(_n_joints);
    for (size_t i = 0; i < _n_joints; ++i)
    {
        _current_joint_pose[i] = msg->position[i];
    }
}

bool NiryoOneTestMotor::runTest(int nb_loops)
{
    using namespace std::chrono_literals;
    enable_test = true;
    
    RCLCPP_INFO(node->get_logger(),"Reset Controller");
    std_msgs::msg::Empty reset_controller_topic;
    reset_stepper_publisher->publish(reset_controller_topic);
    sleep_for(0.05);

    bool state;
    state = playTrajectory(armExtensionTrajectory(pose_start));
    if (!state) {return false;}

    for (int j=0; j<nb_loops; j++)
    {
        for (size_t i = 0; i < _n_joints; ++i)
        {
            if (_joint_has_position_limits[i])
            {
                std::vector<double> pose_lower_limit = pose_start;
                pose_lower_limit[i] = _joint_lower_limits[i];
                state = playTrajectory(armExtensionTrajectory(pose_lower_limit));
                if (!state) {return false;}

                state = playTrajectory(armExtensionTrajectory(pose_start));
                if (!state) {return false;}

                std::vector<double> pose_upper_limit = pose_start;
                pose_upper_limit[i] = _joint_upper_limits[i];
                state = playTrajectory(armExtensionTrajectory(pose_upper_limit));
                if (!state) {return false;}

                state = playTrajectory(armExtensionTrajectory(pose_start));
                if (!state) {return false;}
            }
        }
    }
    enable_test = false;

    return true;
}

void NiryoOneTestMotor::stopTest()
{
    if (enable_test)
    {    
        enable_test = false;
        RCLCPP_INFO(node->get_logger(),"Reset Controller");
        std_msgs::msg::Empty reset_controller_topic;
        reset_stepper_publisher->publish(reset_controller_topic);
    }
    return;
}

bool NiryoOneTestMotor::getJointsLimits()
{
    // Resize vectors
    _joint_names.resize(_n_joints);
    _joint_upper_limits.resize(_n_joints);
    _joint_lower_limits.resize(_n_joints);
    _joint_has_position_limits.resize(_n_joints);

    // Get limits from URDF
    auto urdfModel = new urdf::Model;
    urdf::ModelSharedPtr urdf(urdfModel);
    std::string urdf_str;
    
    if (node->get_parameter("robot_description", urdf_str)) // Check for robot_description in proper namespace
    {
        if (!urdf->initString(urdf_str))
        {
            RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to parse URDF contained in 'robot_description' parameter (namespace: " << node->get_namespace() << ").");
            return false;
        }
    }
    /*
    else if (!urdf->initParam("robot_description")) // Check for robot_description in root
    {
        //ROS_ERROR_STREAM("Failed to parse URDF contained in 'robot_description' parameter");
        return false;
    }
    */ //initParam not supported yet
    for (auto i = 0; i < _n_joints; i++)
    {
        // Joints name
        _joint_names[i]= "joint_" + std::to_string(i+1);

        urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(_joint_names[i]);
        if (urdf_joint)
        {
            if (urdf_joint->type != urdf::Joint::CONTINUOUS)
            {
                _joint_upper_limits[i] = urdf_joint->limits->upper - 0.2;
                _joint_lower_limits[i] = urdf_joint->limits->lower + 0.2;
                _joint_has_position_limits[i] = true;
            }
            else
                _joint_has_position_limits[i] = false;
        }
        else
        {
            return false;
        }
    }
}

void NiryoOneTestMotor::startTrajectory(control_msgs::action::FollowJointTrajectory_Goal_<std::allocator<void>> goal)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(node->get_logger(),"Send trajectory");
    while(!traj_client_->wait_for_action_server(std::chrono::seconds(5))){
        RCLCPP_INFO(node->get_logger(),"Waiting for the joint_trajectory_action server");
    }

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = node->now() + rclcpp::Duration::from_seconds(1.0);
    
    auto result_future = traj_client_->async_send_goal(goal);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
        return;
    }

}
bool NiryoOneTestMotor::playTrajectory(control_msgs::action::FollowJointTrajectory_Goal_<std::allocator<void>> goal)
{
    if (!enable_test)
    {
        return true;
    }
    
    startTrajectory(goal);
    
    std::vector<double> rounded_target;
    rounded_target.resize(_n_joints);

    for (size_t i = 0; i < _n_joints; ++i)
    {
        rounded_target[i] = round(goal.trajectory.points[goal.trajectory.points.size()-1].positions[i]*10)/10;
    }

    sleep_for(0.3);
    for (size_t i = 0; i < _n_joints; ++i)
    {
        double goal_joint = goal.trajectory.points[goal.trajectory.points.size()-1].positions[i];
        if (enable_test && (( _current_joint_pose[i] < goal_joint-0.1) || (_current_joint_pose[i] > goal_joint+0.1)))
        {
            return false;
        }
        
    }

    return true;
}

control_msgs::action::FollowJointTrajectory_Goal_<std::allocator<void>> NiryoOneTestMotor::armExtensionTrajectory(std::vector<double> joint_positions)
{
    //our goal variable
    control_msgs::action::FollowJointTrajectory_Goal_<std::allocator<void>> goal;

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    int ind = 0;
    goal.trajectory.joint_names.resize(_n_joints);
    goal.trajectory.points[ind].positions.resize(_n_joints);
    goal.trajectory.points[ind].velocities.resize(_n_joints);

    for (size_t j = 0; j < _n_joints; ++j)
    {
        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names[j] = _joint_names[j];
        // Positions
        goal.trajectory.points[ind].positions[j] = joint_positions[j];
        // Velocities
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = rclcpp::Duration::from_seconds(3.0);

    return goal;
}
void sleep_for(double seconds){
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds)));
}
