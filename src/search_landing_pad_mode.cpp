#include "px4_vision_hardware_cpp/search_landing_pad_mode.hpp"

SearchLandingPadMode::SearchLandingPadMode(rclcpp::Node & node)
: ModeBase(node, Settings{"Search Landing Pad Mode"})
{
    trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // Get search altitude parameter
    node.get_parameter_or("target_altitude", target_altitude_, 4.0f);

    auto qos = rclcpp::SensorDataQoS();
    
    // Subscribe to the Rover's EKF
    rover_odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
        "/rover/ekf_odom", qos, std::bind(&SearchLandingPadMode::rover_odom_cb, this, std::placeholders::_1));

    // Subscribe to the Vision flag
    tag_visible_sub_ = node.create_subscription<std_msgs::msg::Bool>(
        "/landing/tag_visible_flag", qos, std::bind(&SearchLandingPadMode::tag_visible_cb, this, std::placeholders::_1));
}

void SearchLandingPadMode::rover_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Convert Rover ENU (map frame) to Drone NED
    // ENU X (East) -> NED Y
    // ENU Y (North) -> NED X
    rover_pos_ned_(0) = msg->pose.pose.position.y;
    rover_pos_ned_(1) = msg->pose.pose.position.x;
    rover_odom_received_ = true;
}

void SearchLandingPadMode::tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    tag_visible_ = msg->data;
}

void SearchLandingPadMode::onActivate() 
{
    RCLCPP_INFO(node().get_logger(), "[SEARCH MODE] Activated. Navigating to Rover EKF position.");
}

void SearchLandingPadMode::onDeactivate() 
{
    RCLCPP_WARN(node().get_logger(), "[SEARCH MODE] Deactivated.");
}

void SearchLandingPadMode::updateSetpoint(float dt_s)
{
    // Explicitly tell the compiler we are intentionally not using this variable
    (void)dt_s;

    // Exit Condition: If the camera spots the tag, our job is done!
    if (tag_visible_) 
    {
        RCLCPP_INFO(node().get_logger(), "[SEARCH MODE] Tag spotted! Handing over to Vision Landing Mode.");
        completed(px4_ros2::Result::Success);
        return;
    }

    px4_ros2::TrajectorySetpoint sp;

    // Chasing Condition: Fly to the Rover's global coordinates
    if (rover_odom_received_) 
    {
        sp.withHorizontalPosition(rover_pos_ned_);
        sp.withPositionZ(-target_altitude_);
        
        RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 1000, 
            "[SEARCH MODE] Chasing Rover -> N: %.2f, E: %.2f | Alt: %.2f", 
            rover_pos_ned_(0), rover_pos_ned_(1), target_altitude_);
    } 
    // Failsafe Condition: No Rover Data yet, safely hold altitude
    else 
    {
        sp.withHorizontalVelocity(Eigen::Vector2f(0.0f, 0.0f));
        sp.withPositionZ(-target_altitude_);
        RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000, 
            "[SEARCH MODE] Waiting for Rover EKF data...");
    }

    trajectory_setpoint_->update(sp);
}