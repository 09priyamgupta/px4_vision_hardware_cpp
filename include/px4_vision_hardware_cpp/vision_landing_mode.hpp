#ifndef VISION_LANDING_MODE_HPP
#define VISION_LANDING_MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

// ROS 2 Messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

// Matrix Math
#include <Eigen/Dense>


class VisionLandingMode : public px4_ros2::ModeBase
{
    public:
        explicit VisionLandingMode(rclcpp::Node & node);

        // Overrides from ModeBase
        void onActivate() override;
        void onDeactivate() override;

        // PX4 calls this automatically when the mode is active.
        // This is where the main control logic of the mode will be implemented and 
        // it replaces the 20Hz control timer
        void updateSetpoint(float dt_s) override;

    private:
        // ----------------------- Callbacks ------------------------
        void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        void tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg);
        void mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        // Landing Service callback
        void landing_service_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);

        // ----------------------- State Variables ------------------------
        Eigen::Vector3f drone_pos_ned_{0, 0, 0};
        Eigen::Quaternionf drone_quat_{1.0f, 0.0f, 0.0f, 0.0f};
        Eigen::Vector2f tag_rel_body_{0, 0};        // Front, Right
        float drone_yaw_ned_ = 0.0f;
        float current_target_altitude_ = 4.0f;
        
        bool tag_visible_ = false;
        bool kf_initialized_ = false;
        bool landing_triggered_ = false;

        // --- MPC ---
        Eigen::Vector2f mpc_vel_cmd_{0.0f, 0.0f};

        // --- Kalman Filter (Eigen) ---
        Eigen::Vector4f kf_x_ = Eigen::Vector4f::Zero(); // [x, y, vx, vy]
        Eigen::Matrix4f kf_P_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f F_kf_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix<float, 2, 4> H_kf_;
        Eigen::Matrix4f Q_kf_;
        Eigen::Matrix2f R_kf_;
        Eigen::Vector2f smoothed_tag_vel_ = Eigen::Vector2f::Zero();

        // --- Parameters ---
        float chase_altitude_ = 4.0f;
        float descent_rate_ = 0.3f;
        float touchdown_altitude_ = 0.55f;
        float max_rover_speed_ = 1.0f;

        // --- PX4 Interface Setpoint Object ---
        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;

        // --- ROS 2 Interfaces ---
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_visible_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_cmd_sub_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_state_pub_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr landing_srv_;
};

#endif // VISION_LANDING_MODE_HPP