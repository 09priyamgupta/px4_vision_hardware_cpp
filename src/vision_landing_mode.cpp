#include "px4_vision_hardware_cpp/vision_landing_mode.hpp"
#include <cmath>
#include <algorithm>


VisionLandingMode::VisionLandingMode(rclcpp::Node & node)
: ModeBase(node, Settings{"Vision Landing Mode"})
{
    // Setup the Setpoint Type (Tells PX4 we will send Trajectory setpoints)
    trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // Load Parameters from config yaml file (with defaults)
    if (!node.has_parameter("target_altitude")) node.declare_parameter("target_altitude", 4.0);
    if (!node.has_parameter("descent_rate")) node.declare_parameter("descent_rate", 0.3);
    if (!node.has_parameter("touchdown_altitude")) node.declare_parameter("touchdown_altitude", 0.55);
    if (!node.has_parameter("max_rover_speed")) node.declare_parameter("max_rover_speed", 1.0);

    // Fetch the values
    chase_altitude_ = static_cast<float>(node.get_parameter("target_altitude").as_double());
    descent_rate_ = static_cast<float>(node.get_parameter("descent_rate").as_double());
    touchdown_altitude_ = static_cast<float>(node.get_parameter("touchdown_altitude").as_double());
    max_rover_speed_ = static_cast<float>(node.get_parameter("max_rover_speed").as_double());

    // Initialize Kalman Filter Matrices
    H_kf_ << 1.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 1.0f, 0.0f, 0.0f;

    Q_kf_ = Eigen::Vector4f(0.05f, 0.05f, 0.005f, 0.005f).asDiagonal();
    R_kf_ = Eigen::Vector2f(0.4f, 0.4f).asDiagonal();

    // -------------- ROS 2 Publishers -------------
    mpc_state_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/state", 10);
    debug_error_pub_ = node.create_publisher<std_msgs::msg::Float32>("/mpc/debug/errors", 10);
    debug_ema_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/debug/ema_vel", 10);
    debug_kf_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/debug/kf_state", 10);

    // -------------- ROS 2 Subscriptions & Services -------------
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&VisionLandingMode::odom_cb, this, std::placeholders::_1));
    
    tag_pose_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/apriltag/relative_pose", qos, std::bind(&VisionLandingMode::tag_cb, this, std::placeholders::_1));

    tag_visible_sub_ = node.create_subscription<std_msgs::msg::Bool>(
        "/landing/tag_visible_flag", qos, std::bind(&VisionLandingMode::tag_visible_cb, this, std::placeholders::_1));

    mpc_cmd_sub_ = node.create_subscription<std_msgs::msg::Float32MultiArray>(
    "/mpc/command", 10, std::bind(&VisionLandingMode::mpc_cmd_cb, this, std::placeholders::_1));

    landing_srv_ = node.create_service<std_srvs::srv::Trigger>(
        "/landing/trigger_landing", std::bind(&VisionLandingMode::landing_service_cb, this, std::placeholders::_1, std::placeholders::_2));
}


// ------------------------------------------------------------------------
// ------------------------ Callbacks ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
{
    drone_pos_ned_ << msg->position[0], msg->position[1], msg->position[2];

    // Store the exact IMU attitude to cancel out camera tilt later
    drone_quat_ = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    // Fast Quaternion to Yaw conversion
    drone_yaw_ned_ = atan2(2.0f * (msg->q[0]*msg->q[3] + msg->q[1]*msg->q[2]), 
                           1.0f - 2.0f * (msg->q[2]*msg->q[2] + msg->q[3]*msg->q[3]));
}

void VisionLandingMode::tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg) 
{
    tag_visible_ = msg->data;
    if (!tag_visible_) kf_initialized_ = false;
}

void VisionLandingMode::landing_service_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response) 
{
    (void)request;
    
    landing_triggered_ = true;
    RCLCPP_INFO(node().get_logger(), "[VISION MODE] Landing Triggered! Initiating descent.");
    response->success = true;
}

void VisionLandingMode::tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    // Port of your apriltag_pose_callback
    Eigen::Vector3f p_cam(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // Camera to FRD body frame
    Eigen::Matrix3f R_cam_to_frd;
    R_cam_to_frd << -1, 0, 0,
                     0, 1, 0,
                     0, 0,-1;
                     
    Eigen::Vector3f v_frd = R_cam_to_frd * p_cam;
    tag_rel_body_ << v_frd(0), v_frd(1);

    // Apply the Drone's IMU Quaternion to instantly cancel out camera pitch/roll!
    Eigen::Vector3f v_ned = drone_quat_ * v_frd;

    // Calculate Absolute Tag Position
    Eigen::Vector2f tag_abs_ned = Eigen::Vector2f(drone_pos_ned_(0) + v_ned(0), drone_pos_ned_(1) + v_ned(1));

    if (!kf_initialized_) 
    {
        kf_x_ << tag_abs_ned(0), tag_abs_ned(1), 0.0f, 0.0f;
        kf_initialized_ = true;
        return;
    }

    // --- Kalman Filter Update (Executes when tag is seen) ---
    Eigen::Vector2f z_meas = tag_abs_ned;
    Eigen::Vector2f y_res = z_meas - (H_kf_ * kf_x_);
    Eigen::Matrix2f S = H_kf_ * kf_P_ * H_kf_.transpose() + R_kf_;
    Eigen::Matrix<float, 4, 2> K = kf_P_ * H_kf_.transpose() * S.inverse();
    
    kf_x_ = kf_x_ + (K * y_res);
    kf_P_ = (Eigen::Matrix4f::Identity() - K * H_kf_) * kf_P_;
}

void VisionLandingMode::mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
{
    if (msg->data.size() >= 2) 
    {
        mpc_vel_cmd_(0) = msg->data[0]; // cmd_v_north
        mpc_vel_cmd_(1) = msg->data[1]; // cmd_v_east
    }
}


// ------------------------------------------------------------------------
// ------------------------ Mode Lifecycle Callbacks ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::onActivate() 
{
    RCLCPP_INFO(node().get_logger(), "[VISION MODE] Active! Assuming control.");
    kf_initialized_ = false;
    current_target_altitude_ = chase_altitude_;
}

void VisionLandingMode::onDeactivate() 
{
    RCLCPP_WARN(node().get_logger(), "[VISION MODE] Deactivated.");
}


// ------------------------------------------------------------------------
// ------------------------ Main Control Logic ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::updateSetpoint(float dt_s) 
{
    float actual_altitude = -drone_pos_ned_(2);  // NED Z is negative-up

    // ----------------- DEBUG LOGGING -----------------
    RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 500, 
        "[DEBUG ALTITUDE] Actual Alt: %.2fm | Target Alt: %.2fm | NED Z: %.2f", 
        actual_altitude, current_target_altitude_, drone_pos_ned_(2));

    // Safety Hover if Tag is lost
    if (!tag_visible_ || !kf_initialized_) 
    {
        // Use the safe struct for hover to completely avoid the NaN bug!
        px4_ros2::TrajectorySetpoint sp;
        sp.withHorizontalVelocity(Eigen::Vector2f(0.0f, 0.0f)); // Brake and hold XY
        sp.withPositionZ(-current_target_altitude_);            // Hold Altitude

        RCLCPP_DEBUG_THROTTLE(node().get_logger(), *node().get_clock(), 1000, 
            "[DEBUG] Tag lost. Sending Safety Hover Setpoint.");

        trajectory_setpoint_->update(sp);
        return;
    }

    // Run Kalman Filter Prediction Step
    F_kf_(0, 2) = dt_s;
    F_kf_(1, 3) = dt_s;
    kf_x_ = F_kf_ * kf_x_;
    kf_P_ = F_kf_ * kf_P_ * F_kf_.transpose() + Q_kf_;

    // EMA Filter & Clamp Velocity
    Eigen::Vector2f raw_tag_vel(kf_x_(2), kf_x_(3));
    float alpha = 0.15f;
    smoothed_tag_vel_ = (alpha * raw_tag_vel) + ((1.0f - alpha) * smoothed_tag_vel_);
    
    float vn = std::clamp(smoothed_tag_vel_(0), -max_rover_speed_, max_rover_speed_);
    float ve = std::clamp(smoothed_tag_vel_(1), -max_rover_speed_, max_rover_speed_);

    // 1. Calculate FOV radius and Lookahead
    float safe_alt = std::max(0.2f, actual_altitude);
    float fov_radius = safe_alt * std::tan(20.0f * M_PI / 180.0f) * 0.8f;
    float t_lookahead = std::max(0.0f, (actual_altitude - touchdown_altitude_) * 0.5f);
    
    float offset_north = vn * t_lookahead;
    float offset_east = ve * t_lookahead;
    
    float lead_offset = std::hypot(offset_north, offset_east);
    float max_allowed_offset = fov_radius * 0.7f;
    
    if (lead_offset > max_allowed_offset) 
    {
        float scale = max_allowed_offset / lead_offset;
        offset_north *= scale;
        offset_east *= scale;
    }

    // 2. Apply predictive offset
    float future_tag_north = kf_x_(0) + offset_north;
    float future_tag_east  = kf_x_(1) + offset_east;

    // 3. Feed the MPC the predictive error
    Eigen::Vector2f drone_in_tag_pos(drone_pos_ned_(0) - future_tag_north, drone_pos_ned_(1) - future_tag_east);
    float horiz_error = drone_in_tag_pos.norm();

    // Publish Tracking Error
    std_msgs::msg::Float32 error_msg;
    error_msg.data = horiz_error;
    debug_error_pub_->publish(error_msg);

    // Publish Smoothed EMA Velocity [Vn, Ve]
    std_msgs::msg::Float32MultiArray ema_msg;
    ema_msg.data = {smoothed_tag_vel_(0), smoothed_tag_vel_(1)};
    debug_ema_pub_->publish(ema_msg);

    // Publish Internal Kalman Filter State [x, y, vx, vy]
    std_msgs::msg::Float32MultiArray kf_msg;
    kf_msg.data = {kf_x_(0), kf_x_(1), kf_x_(2), kf_x_(3)};
    debug_kf_pub_->publish(kf_msg);

    // ------------------- Conditional Landing Logic -------------------
    float cmd_v_down = 0.0f; // Default Z-Velocity constraint (Hold Alt)

    if (landing_triggered_) 
    {
        float allowed_error = std::max(0.2f, 0.4f * actual_altitude);
        if (horiz_error < allowed_error) 
        {
            current_target_altitude_ -= descent_rate_ * dt_s;
            cmd_v_down = descent_rate_; // Set downward velocity constraint
        }
        current_target_altitude_ = std::max(touchdown_altitude_, current_target_altitude_);

        if (actual_altitude <= (touchdown_altitude_ + 0.1f) && horiz_error < 0.15f) 
        {
            RCLCPP_WARN(node().get_logger(), "TOUCHDOWN DETECTED! Completing Mode.");
            completed(px4_ros2::Result::Success); 
            return;
        }
    }

    // THE ROS 2 BRIDGE
    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {drone_in_tag_pos(0), drone_in_tag_pos(1), vn, ve, actual_altitude};
    mpc_state_pub_->publish(state_msg);

    // Grab the latest command we received from the Python node
    float cmd_v_north = mpc_vel_cmd_(0); 
    float cmd_v_east  = mpc_vel_cmd_(1);

    // Yaw Control
    float yaw_rate_cmd = 0.0f;
    if (std::hypot(vn, ve) > 0.15f) 
    {
        float target_yaw = atan2(ve, vn);
        float yaw_error = target_yaw - drone_yaw_ned_;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error)); // wrap to -pi to pi
        yaw_rate_cmd = std::clamp(0.6f * yaw_error, -0.4f, 0.4f);
    }

    // 7. PUBLISH TO PX4 (Using the TrajectorySetpoint Struct)
    px4_ros2::TrajectorySetpoint sp;
    
    // Always command horizontal velocity and yaw rate
    sp.withHorizontalVelocity(Eigen::Vector2f(cmd_v_north, cmd_v_east));
    sp.withYawRate(yaw_rate_cmd);

    // Toggle between Z-Position (Chase) and Z-Velocity (Landing)
    if (landing_triggered_) 
    {
        // Pure velocity downward for landing
        sp.withVelocityZ(cmd_v_down);
    } 
    else 
    {
        // Hybrid: XY Velocity + Z Position Hold for chasing
        sp.withPositionZ(-current_target_altitude_);
    }

    // --- DEBUGGING --- 
    RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 500, 
        "[TRACKING SETPOINT] Vx(N): %.2f | Vy(E): %.2f | Z_pos: %.2f | Z_vel: %.2f | YawRate: %.2f", 
        cmd_v_north, 
        cmd_v_east, 
        (landing_triggered_ ? NAN : -current_target_altitude_), 
        (landing_triggered_ ? cmd_v_down : NAN),
        yaw_rate_cmd);

    // Send the cleanly built struct to PX4!
    trajectory_setpoint_->update(sp);
}