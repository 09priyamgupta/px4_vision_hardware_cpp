#ifndef VISION_LANDING_MODE_HPP
#define VISION_LANDING_MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
<<<<<<< HEAD
=======

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
>>>>>>> a79a21d (final hardware with csv logger)

// ROS 2 Messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int32.hpp>

// Matrix Math
#include <Eigen/Dense>
#include <deque>
#include <cmath>

/**
 * @brief Custom PX4 Flight Mode for Vision-Based Tracking and Landing.
 * 
 * This class inherits from px4_ros2::ModeBase. It subscribes to AprilTag
 * detections, filters the target's position/velocity using a Kalman Filter,
 * communicates tracking errors to an external Model Predictive Controller (MPC),
 * and feeds the resulting velocity commands into PX4 via Trajectory Setpoints.
 */

/**
 * @brief Custom PX4 Flight Mode for Vision-Based Tracking and Landing.
 * 
 * This class inherits from px4_ros2::ModeBase. It subscribes to AprilTag
 * detections, filters the target's position/velocity using a Kalman Filter,
 * communicates tracking errors to an external Model Predictive Controller (MPC),
 * and feeds the resulting velocity commands into PX4 via Trajectory Setpoints.
 */

<<<<<<< HEAD

=======
>>>>>>> a79a21d (final hardware with csv logger)
// ==============================================================================
//                  TUNING CONSTANTS & HARDCODED VALUES
// ==============================================================================
namespace Config 
{
    // --- Kalman Filter Tuning ---
<<<<<<< HEAD
    constexpr float KF_Q_POS = 0.05f;            // Process noise variance for Position
    constexpr float KF_Q_VEL = 0.005f;           // Process noise variance for Velocity
    constexpr float KF_R_POS = 0.4f;             // Measurement noise variance for Tag Detection
=======
    constexpr float KF_Q_POS = 0.01f;            // Process noise variance for Position: Trust the physics model's position heavily
    constexpr float KF_Q_VEL = 0.002f;           // Process noise variance for Velocity: Assume the rover drives smoothly; aggressively reject velocity spikes
    constexpr float KF_R_POS = 0.05f;            // Measurement noise variance for Tag Detection: Trust the camera! 0.05 assumes ~22cm of max optical noise/blur
>>>>>>> a79a21d (final hardware with csv logger)

    // --- RC Fallback Tuning (Safety Hover) ---
    constexpr float RC_DEADBAND = 0.08f;         // Deadband for manual stick inputs
    constexpr float RC_MAX_XY_VEL = 2.0f;        // Max horizontal speed (m/s) from sticks
    constexpr float RC_MAX_Z_VEL = 1.0f;         // Max descent speed (m/s) from sticks
    constexpr float RC_MAX_YAW_RATE = 0.5f;      // Max yaw rate (rad/s) from sticks

    // --- Tracking & Prediction Tuning ---
<<<<<<< HEAD
    constexpr float EMA_ALPHA = 0.15f;           // Exponential Moving Average weight for velocity smoothing
=======
    constexpr float EMA_ALPHA = 0.3f;            // Exponential Moving Average weight for velocity smoothing
>>>>>>> a79a21d (final hardware with csv logger)
    constexpr float FOV_ANGLE_DEG = 20.0f;       // Camera Field of View half-angle in degrees
    constexpr float FOV_SAFE_SCALE = 0.8f;       // Usable fraction of the FOV (margin of safety)
    constexpr float LOOKAHEAD_SCALE = 0.5f;      // Scales how many seconds ahead to look based on altitude difference
    constexpr float MAX_LEAD_SCALE = 0.7f;       // Max allowed predictive lead as a fraction of the safe FOV radius
    
    // --- Auto-Yaw Control ---
    constexpr float YAW_MOVE_THRESHOLD = 0.15f;  // Min rover speed (m/s) required to trigger auto-yaw tracking
    constexpr float YAW_P_GAIN = 0.6f;           // Proportional gain for heading correction
    constexpr float YAW_RATE_MAX = 0.4f;         // Maximum auto-yaw rate (rad/s)

    // --- Landing Constraints ---
<<<<<<< HEAD
    constexpr float LANDING_ERR_MIN = 0.2f;      // Minimum absolute horizontal error (m) tolerated to descend
    constexpr float LANDING_ERR_SCALE = 0.4f;    // Altitude multiplier for acceptable horizontal error 
    constexpr float TOUCHDOWN_ALT_MARGIN = 0.1f; // Altitude margin (m) above touchdown target to declare sequence complete
    constexpr float TOUCHDOWN_ERR_MAX = 0.15f;   // Max horizontal error (m) acceptable for final touchdown check
=======
    constexpr float LANDING_ERR_MIN = 0.3f;      // Minimum absolute horizontal error (m) tolerated to descend
    constexpr float LANDING_ERR_SCALE = 0.5f;    // Altitude multiplier for acceptable horizontal error 
    constexpr float TOUCHDOWN_ALT_MARGIN = 0.1f; // Altitude margin (m) above touchdown target to declare sequence complete
    constexpr float TOUCHDOWN_ERR_MAX = 0.3f;   // Max horizontal error (m) acceptable for final touchdown check
>>>>>>> a79a21d (final hardware with csv logger)
}


// ==============================================================================
//              Vision-Based Tracking and Landing Class
// ==============================================================================
class VisionLandingMode : public px4_ros2::ModeBase
{
    public:
        explicit VisionLandingMode(rclcpp::Node & node);

        // Overrides from ModeBase
        void onActivate() override;
        void onDeactivate() override;

        /**
         * @brief Core control loop, called automatically by PX4 at high frequency (e.g., 50Hz).
         * @param dt_s Delta time in seconds since the last call.
         */
        void updateSetpoint(float dt_s) override;

    private:
<<<<<<< HEAD
        // ----------------------- Callbacks ------------------------
        void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        void tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg);
        void mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        // ------------------- Landing Service callback -------------------
        void landing_service_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);

        // ----------------------- Math Utilities ------------------------
        float wrap_pi(float angle);                 // Wrap angle to [-pi, pi]
        float with_deadband(float x, float db);     // Apply deadband to a value

        // ----------------------- State Variables ------------------------
        // Drone State
        Eigen::Vector3f drone_pos_ned_{0, 0, 0};                // Current position in North-East-Down (NED) frame
        Eigen::Quaternionf drone_quat_{1.0f, 0.0f, 0.0f, 0.0f}; // Current IMU orientation
        float drone_yaw_ned_ = 0.0f;                            // Current heading in radians
        
        // Setpoint Tracking
        Eigen::Vector3f pos_sp_ned_{Eigen::Vector3f::Zero()};   // Fallback Goto position setpoint
        float current_target_altitude_ = 4.0f;                  // Dynamic altitude target (changes during descent)
        float yaw_sp_rad_ = 0.0f;                               // Desired yaw setpoint
        
        // Mode Flags
        bool tag_visible_ = false;       // True if the tag is currently in the camera FOV
        bool kf_initialized_ = false;    // True if the Kalman filter has an initial state
        bool landing_triggered_ = false; // True if the landing sequence has been initiated via ROS service

        // Target Tracking
        Eigen::Vector2f tag_rel_body_{0, 0};        // Raw tag position relative to camera (Front, Right)
        Eigen::Vector2f smoothed_tag_vel_ = Eigen::Vector2f::Zero(); // EMA smoothed velocity of the rover

        // External Controller (MPC) Commands
        Eigen::Vector2f mpc_vel_cmd_{0.0f, 0.0f};   // Velocity commands [V_north, V_east] received from Python MPC node
        
        // --- Kalman Filter Matrices (2D Position/Velocity tracking) ---
        Eigen::Vector4f kf_x_ = Eigen::Vector4f::Zero(); // State Vector: [pos_x, pos_y, vel_x, vel_y]
        Eigen::Matrix4f kf_P_ = Eigen::Matrix4f::Identity(); // State Covariance
        Eigen::Matrix4f F_kf_ = Eigen::Matrix4f::Identity(); // State Transition Model (updated dynamically with dt)
        Eigen::Matrix<float, 2, 4> H_kf_;                    // Observation Model
        Eigen::Matrix4f Q_kf_;                               // Process Noise Covariance
        Eigen::Matrix2f R_kf_;                               // Measurement Noise Covariance

        // --- Mode Parameters ---
        float chase_altitude_ = 4.0f;      // Default altitude to hold while tracking the rover
        float descent_rate_ = 0.3f;        // Z-velocity (m/s) during the landing phase
        float touchdown_altitude_ = 0.55f; // Altitude threshold to declare landing complete
        float max_rover_speed_ = 1.0f;     // Safety clamp for velocity tracking

        // --- PX4 Interface Setpoint Objects ---
        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_; // For MPC velocity control
        std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint_;            // For safety hover (position control)
        std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input_;   // For RC stick fallbacks
=======
        // --- TF2 ---
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
>>>>>>> a79a21d (final hardware with csv logger)

        // --- ROS 2 Subscribers ---
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_visible_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_cmd_sub_;
        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_stick_sub_;

        // --- ROS 2 Publishers ---
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr debug_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_ema_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_kf_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr custom_state_pub_;

        // --- ROS 2 Service ---
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr landing_srv_;
        
        // ----------------------- Callbacks ------------------------
        void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        void tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg);
        void mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void visual_odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

        // ------------------- Landing Service callback -------------------
        void landing_service_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);

        // ----------------------- Math Utilities ------------------------
        float wrap_pi(float angle);                 // Wrap angle to [-pi, pi]
        float with_deadband(float x, float db);     // Apply deadband to a value

        // ----------------------- State Variables ------------------------
        int lost_tag_count_ = 0;
        
        // Drone State
        Eigen::Vector3f drone_pos_ned_{0, 0, 0};                
        Eigen::Quaternionf drone_quat_{1.0f, 0.0f, 0.0f, 0.0f}; // Current IMU orientation
        float drone_yaw_ned_ = 0.0f;                            // Current heading in radians
        Eigen::Vector3f drone_vel_ned_;

        // State variables for the MOCAP PX4 Bridge input
        Eigen::Vector3f bridge_pos_ned_{0.0f, 0.0f, 0.0f};
        float bridge_yaw_ned_ = 0.0f;
        
        // Setpoint Tracking
        Eigen::Vector3f pos_sp_ned_{Eigen::Vector3f::Zero()};   // Fallback Goto position setpoint
        float current_target_altitude_ = 4.0f;                  // Dynamic altitude target (changes during descent)
        float yaw_sp_rad_ = 0.0f;                               // Desired yaw setpoint
        
        // Mode Flags
        bool tag_visible_ = false;       // True if the tag is currently in the camera FOV
        bool kf_initialized_ = false;    // True if the Kalman filter has an initial state
        bool landing_triggered_ = false; // True if the landing sequence has been initiated via ROS service

        // Target Tracking
        Eigen::Vector2f tag_rel_body_{0, 0};        // Raw tag position relative to camera (Front, Right)
        Eigen::Vector2f smoothed_tag_vel_ = Eigen::Vector2f::Zero(); // EMA smoothed velocity of the rover
        Eigen::Vector3f tag_offset_ned_{0.0f, 0.0f, 0.0f};

        // External Controller (MPC) Commands
        Eigen::Vector2f mpc_vel_cmd_{0.0f, 0.0f};   // Velocity commands [V_north, V_east] received from Python MPC node
        
        // --- Kalman Filter Matrices (2D Position/Velocity tracking) ---
        Eigen::Vector4f kf_x_ = Eigen::Vector4f::Zero(); // State Vector: [pos_x, pos_y, vel_x, vel_y]
        Eigen::Matrix4f kf_P_ = Eigen::Matrix4f::Identity(); // State Covariance
        Eigen::Matrix4f F_kf_ = Eigen::Matrix4f::Identity(); // State Transition Model (updated dynamically with dt)
        Eigen::Matrix<float, 2, 4> H_kf_;                    // Observation Model
        Eigen::Matrix4f Q_kf_;                               // Process Noise Covariance
        Eigen::Matrix2f R_kf_;                               // Measurement Noise Covariance

        // --- Mode Parameters ---
        float chase_altitude_ = 4.0f;      // Default altitude to hold while tracking the rover
        float descent_rate_ = 0.3f;        // Z-velocity (m/s) during the landing phase
        float touchdown_altitude_ = 0.55f; // Altitude threshold to declare landing complete
        float max_rover_speed_ = 1.0f;     // Safety clamp for velocity tracking

        // --- PX4 Interface Setpoint Objects ---
        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_; // For MPC velocity control
        std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint_;            // For safety hover (position control)
        std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input_;   // For RC stick fallbacks

        struct OdomRecord {
                                rclcpp::Time stamp;
                                Eigen::Vector3f pos_ned;
                                Eigen::Quaternionf quat;
                            };

        std::deque<OdomRecord> odom_history_; // Buffer of recent odometry for latency compensation
};

#endif // VISION_LANDING_MODE_HPP