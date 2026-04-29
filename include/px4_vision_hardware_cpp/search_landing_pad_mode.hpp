#ifndef SEARCH_LANDING_PAD_MODE_HPP
#define SEARCH_LANDING_PAD_MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>


class SearchLandingPadMode : public px4_ros2::ModeBase
{
    public:
        explicit SearchLandingPadMode(rclcpp::Node & node);

        // Overrides from ModeBase
        void onActivate() override;
        void onDeactivate() override;

        // PX4 calls this automatically when the mode is active.
        // This is where the main control logic of the mode will be implemented and 
        // it replaces the 20Hz control timer
        void updateSetpoint(float dt_s) override;

    private:
        // ----------------------- Callbacks ------------------------
        void rover_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
        void tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg);

        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rover_odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_visible_sub_;

        Eigen::Vector2f rover_pos_ned_{0.0f, 0.0f};
        bool rover_odom_received_ = false;
        bool tag_visible_ = false;
        
        float target_altitude_ = 4.0f;
};

#endif // SEARCH_LANDING_PAD_MODE_HPP