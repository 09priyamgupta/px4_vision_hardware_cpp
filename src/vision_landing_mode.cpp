#include "px4_vision_hardware_cpp/vision_landing_mode.hpp"
#include <cmath>
#include <algorithm>


VisionLandingMode::VisionLandingMode(rclcpp::Node & node)
: ModeBase(node, Settings{"Vision Landing Mode"})
{
    // --------------------- Setup PX4 Setpoint Interfaces ---------------------
    // TrajectorySetpoint is used for high-frequency MPC velocity tracking.
    trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // GotoSetpoint is used for smooth position holds when safety hover is triggered.
    _goto_setpoint_ = std::make_shared<px4_ros2::GotoSetpointType>(*this); 

    // ManualControlInput allows us to read RC stick inputs during fallback states.
    _manual_control_input_ = std::make_shared<px4_ros2::ManualControlInput>(*this);

    // -------- Load Parameters from config yaml file (with defaults) --------
    if (!node.has_parameter("target_altitude")) node.declare_parameter("target_altitude", 4.0);
    if (!node.has_parameter("descent_rate")) node.declare_parameter("descent_rate", 0.3);
    if (!node.has_parameter("touchdown_altitude")) node.declare_parameter("touchdown_altitude", 0.55);
    if (!node.has_parameter("max_rover_speed")) node.declare_parameter("max_rover_speed", 1.0);

    // Fetch the values
    chase_altitude_ = static_cast<float>(node.get_parameter("target_altitude").as_double());
    descent_rate_ = static_cast<float>(node.get_parameter("descent_rate").as_double());
    touchdown_altitude_ = static_cast<float>(node.get_parameter("touchdown_altitude").as_double());
    max_rover_speed_ = static_cast<float>(node.get_parameter("max_rover_speed").as_double());

    // --------------------- Initialize Kalman Filter Matrices ---------------------
    // Observation Matrix (We only measure Position X and Y, not Velocity)
    H_kf_ << 1.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 1.0f, 0.0f, 0.0f;

    // Process Noise (Q) and Measurement Noise (R)
    Q_kf_ = Eigen::Vector4f(Config::KF_Q_POS, Config::KF_Q_POS, Config::KF_Q_VEL, Config::KF_Q_VEL).asDiagonal();
    R_kf_ = Eigen::Vector2f(Config::KF_R_POS, Config::KF_R_POS).asDiagonal();
<<<<<<< HEAD
=======

    // --------------------- Setup TF2 Listener for Frame Transforms ---------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Define QoS profile for low-latency sensor/control data (Queue size = 1, Best Effort)
    auto qos = rclcpp::SensorDataQoS();
>>>>>>> a79a21d (final hardware with csv logger)

    // -------------- ROS 2 Publishers -------------
    mpc_state_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/state", qos);
    debug_error_pub_ = node.create_publisher<std_msgs::msg::Float32>("/mpc/debug/errors", qos);
    debug_ema_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/debug/ema_vel", qos);
    debug_kf_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>("/mpc/debug/kf_state", qos);
    custom_state_pub_ = node.create_publisher<std_msgs::msg::Int32>("/landing/custom_mission_state", qos);

    // -------------- ROS 2 Subscriptions & Services -------------
    odom_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&VisionLandingMode::odom_cb, this, std::placeholders::_1));
    
    tag_pose_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/apriltag/relative_pose", qos, std::bind(&VisionLandingMode::tag_cb, this, std::placeholders::_1));

    tag_visible_sub_ = node.create_subscription<std_msgs::msg::Bool>(
        "/landing/tag_visible_flag", qos, std::bind(&VisionLandingMode::tag_visible_cb, this, std::placeholders::_1));

    mpc_cmd_sub_ = node.create_subscription<std_msgs::msg::Float32MultiArray>(
    "/mpc/command", qos, std::bind(&VisionLandingMode::mpc_cmd_cb, this, std::placeholders::_1));

    visual_odom_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry", qos, std::bind(&VisionLandingMode::visual_odom_cb, this, std::placeholders::_1));

    landing_srv_ = node.create_service<std_srvs::srv::Trigger>(
        "/landing/trigger_landing", std::bind(&VisionLandingMode::landing_service_cb, this, std::placeholders::_1, std::placeholders::_2));
}


// ------------------------------------------------------------------------
// ------------------------ Callbacks ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
{
    // Update drone position in NED frame
    drone_pos_ned_ << msg->position[0], msg->position[1], msg->position[2];
    drone_vel_ned_ << msg->velocity[0], msg->velocity[1], msg->velocity[2];

    // Store IMU orientation to compensate for camera tilt later
    drone_quat_ = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    // Fast Quaternion to Yaw (Heading) conversion
    drone_yaw_ned_ = atan2(2.0f * (msg->q[0]*msg->q[3] + msg->q[1]*msg->q[2]), 
                           1.0f - 2.0f * (msg->q[2]*msg->q[2] + msg->q[3]*msg->q[3]));
    drone_yaw_ned_ = wrap_pi(drone_yaw_ned_);
<<<<<<< HEAD
=======

    // --- Store Odometry History ---
    odom_history_.push_back({node().get_clock()->now(), drone_pos_ned_, drone_quat_});
    
    // Keep a rolling buffer of the last 100 messages (~2 seconds of data at 50Hz)
    if (odom_history_.size() > 100) 
    {
        odom_history_.pop_front();
    }
}

void VisionLandingMode::visual_odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
{
    // Store the raw NED position being sent to PX4
    bridge_pos_ned_ << msg->position[0], msg->position[1], msg->position[2];

    // Convert the Bridge's FRD quaternion to Yaw
    bridge_yaw_ned_ = atan2(2.0f * (msg->q[0]*msg->q[3] + msg->q[1]*msg->q[2]), 
                            1.0f - 2.0f * (msg->q[2]*msg->q[2] + msg->q[3]*msg->q[3]));
    bridge_yaw_ned_ = wrap_pi(bridge_yaw_ned_);
>>>>>>> a79a21d (final hardware with csv logger)
}

void VisionLandingMode::tag_visible_cb(const std_msgs::msg::Bool::SharedPtr msg) 
{
    tag_visible_ = msg->data;
<<<<<<< HEAD
    // Reset KF flag if tag is lost so it re-initializes on the next detection
    if (!tag_visible_) kf_initialized_ = false;
=======
    if (!tag_visible_) 
    {
        lost_tag_count_++;
        if (lost_tag_count_ > 3) kf_initialized_ = false;  // require 3 consecutive losses
    } 
    else 
    {
        lost_tag_count_ = 0;
    }
>>>>>>> a79a21d (final hardware with csv logger)
}

void VisionLandingMode::landing_service_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response) 
{
    landing_triggered_ = true;
    RCLCPP_INFO(node().get_logger(), "[VISION MODE] Landing Triggered! Initiating descent.");
    response->success = true;
}

void VisionLandingMode::tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
<<<<<<< HEAD
    // 1. Extract raw camera pose
    Eigen::Vector3f p_cam(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // 2. Transform: Camera Frame to FRD (Front-Right-Down) Body Frame
    Eigen::Matrix3f R_cam_to_frd;
    R_cam_to_frd << -1, 0, 0,
                     0, 1, 0,
                     0, 0,-1;
                     
    Eigen::Vector3f v_frd = R_cam_to_frd * p_cam;
    tag_rel_body_ << v_frd(0), v_frd(1);

    // 3. Transform: FRD to NED World Frame
    // Applying the Drone's IMU Quaternion directly cancels out camera pitch/roll dynamics.
    Eigen::Vector3f v_ned = drone_quat_ * v_frd;

    // 4. Calculate Absolute Tag Position in NED
    Eigen::Vector2f tag_abs_ned = Eigen::Vector2f(drone_pos_ned_(0) + v_ned(0), drone_pos_ned_(1) + v_ned(1));

    // 5. Initialize Kalman Filter if this is the first detection
=======
    double latency_ms = (node().get_clock()->now() - msg->header.stamp).seconds() * 1000.0;
    
    RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 1000, "PIPELINE LATENCY: %.1f ms", latency_ms);

    // 1. Look up historical drone pose (Where was the drone when the photo was taken?)
    rclcpp::Time tag_time = msg->header.stamp;
    Eigen::Vector3f hist_pos = drone_pos_ned_;
    Eigen::Quaternionf hist_quat = drone_quat_;
    
    double min_dt = 1e9;
    for (const auto& record : odom_history_) 
    {
        double dt = std::abs((record.stamp - tag_time).seconds());
        if (dt < min_dt) 
        {
            min_dt = dt;
            hist_pos = record.pos_ned;
            hist_quat = record.quat;
        }
    }

    // 2. We bypass manual math and let TF2 handle the camera tilt and mounting offsets.
    // The incoming msg is in the camera's optical frame. We just need to transform it
    // to the drone's base_link frame (FLU).
    geometry_msgs::msg::PoseStamped pose_in_flu_body;
    try 
    {
        tf_buffer_->transform(*msg, pose_in_flu_body, "x500_2_priyam", tf2::durationFromSec(0.1));
    } 
    catch (const tf2::TransformException & ex) 
    {
        RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000, "TF body transform failed: %s", ex.what());
        return;
    }

    // 3. Convert the FLU body offset to FRD body offset
    Eigen::Vector3f v_body_frd(
        pose_in_flu_body.pose.position.x, 
        -pose_in_flu_body.pose.position.y,
        -pose_in_flu_body.pose.position.z 
    );

    // 4. Rotate the FRD offset by the historical drone attitude to get the NED offset
    tag_offset_ned_ = hist_quat * v_body_frd;

    // 5. Add the rotated offset to the historical NED position
    Eigen::Vector2f tag_abs_ned(
        hist_pos(0) + tag_offset_ned_.x(),
        hist_pos(1) + tag_offset_ned_.y()
    );

    // 6. Initialize or Update Absolute Kalman Filter
>>>>>>> a79a21d (final hardware with csv logger)
    if (!kf_initialized_) 
    {
        kf_x_ << tag_abs_ned(0), tag_abs_ned(1), 0.0f, 0.0f;
        kf_P_ = Eigen::Matrix4f::Identity() * 0.1f; 
        kf_initialized_ = true;
        return;
    }

<<<<<<< HEAD
    // 6. Kalman Filter Update Step (Correction based on measurement)
    Eigen::Vector2f z_meas = tag_abs_ned;
=======
    // INNOVATION GATING FILTER (Outlier Rejection)
    // Rejects impossible vision measurement jumps per frame
    if (kf_initialized_) 
    {
        float innovation_n = tag_abs_ned(0) - kf_x_(0);
        float innovation_e = tag_abs_ned(1) - kf_x_(1);
        float jump_distance = std::hypot(innovation_n, innovation_e);

        // If the tag appears to have teleported more than 1.5 meters in a single frame, drop it.
        if (jump_distance > 1.5f) 
        {
            RCLCPP_ERROR_THROTTLE(node().get_logger(), *node().get_clock(), 500,
                "[FILTER REJECTION] Dropped anomalous target vision jump: %.2f meters!", jump_distance);
            return; // Exit safely without corrupting the Kalman Filter state vector
        }
    }

    Eigen::Vector2f z_meas = tag_abs_ned; // Feed pure absolute position to the filter
>>>>>>> a79a21d (final hardware with csv logger)
    Eigen::Vector2f y_res = z_meas - (H_kf_ * kf_x_);
    Eigen::Matrix2f S = H_kf_ * kf_P_ * H_kf_.transpose() + R_kf_;
    Eigen::Matrix<float, 4, 2> K = kf_P_ * H_kf_.transpose() * S.inverse();
    
    kf_x_ = kf_x_ + (K * y_res);                                // Update state estimate
    kf_P_ = (Eigen::Matrix4f::Identity() - K * H_kf_) * kf_P_;  // Update covariance
}

void VisionLandingMode::mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
{
    // Receive optimized velocity targets from the external MPC
    if (msg->data.size() >= 2) 
    {
<<<<<<< HEAD
        mpc_vel_cmd_(0) = msg->data[0];     // cmd_v_north
        mpc_vel_cmd_(1) = msg->data[1];     // cmd_v_east
=======
        mpc_vel_cmd_ << msg->data[0], msg->data[1];     // [cmd_v_north, cmd_v_east]
>>>>>>> a79a21d (final hardware with csv logger)
    }
}

// ------------------------------------------------------------------------
// ------------------------ Utility Functions -----------------------------
// ------------------------------------------------------------------------
float VisionLandingMode::wrap_pi(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float VisionLandingMode::with_deadband(float x, float db)
{
    // Applies a deadband and smoothly scales the remaining input
    if (std::fabs(x) < db) return 0.0f;
    const float s = (std::fabs(x) - db) / (1.0f - db);
    return (x > 0.0f ? s : -s);
}

// ------------------------------------------------------------------------
// ------------------------ Utility Functions -----------------------------
// ------------------------------------------------------------------------
float VisionLandingMode::wrap_pi(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float VisionLandingMode::with_deadband(float x, float db)
{
    // Applies a deadband and smoothly scales the remaining input
    if (std::fabs(x) < db) return 0.0f;
    const float s = (std::fabs(x) - db) / (1.0f - db);
    return (x > 0.0f ? s : -s);
}


// ------------------------------------------------------------------------
// ------------------------ Mode Lifecycle Callbacks ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::onActivate() 
{
    RCLCPP_INFO(node().get_logger(), "[VISION MODE] Active! Assuming control.");
    kf_initialized_ = false;
<<<<<<< HEAD
    current_target_altitude_ = chase_altitude_;
=======
    lost_tag_count_ = 0;
    current_target_altitude_ = -drone_pos_ned_(2);
>>>>>>> a79a21d (final hardware with csv logger)
    pos_sp_ned_ = drone_pos_ned_;   // Initialize safety hover setpoint to current location
}

void VisionLandingMode::onDeactivate() 
{
<<<<<<< HEAD
    RCLCPP_WARN(node().get_logger(), "[VISION MODE] Deactivated.");
=======
    RCLCPP_WARN(node().get_logger(), 
        "[VISION MODE] Deactivated! (Common triggers: RC stick override > deadband, mode switch toggled, or PX4 failsafe)");
>>>>>>> a79a21d (final hardware with csv logger)
}

// ------------------------------------------------------------------------
// ------------------------ Main Control Logic ------------------------
// ------------------------------------------------------------------------
void VisionLandingMode::updateSetpoint(float dt_s) 
{
    float actual_altitude = -drone_pos_ned_(2);  // NED Z is negative-up
    px4_ros2::TrajectorySetpoint sp;

    Eigen::Vector2f error_vector(0.0f, 0.0f);
    float horiz_error = 0.0f;

<<<<<<< HEAD
=======
    const char* state_str;
    std_msgs::msg::Int32 custom_state_msg; 

    if (!kf_initialized_) { 
        state_str = "STATE1_HOVER"; 
        custom_state_msg.data = 1; 
    }
    else if (landing_triggered_) { 
        state_str = "STATE3_LAND"; 
        custom_state_msg.data = 3; 
    } 
    else if (!tag_visible_ && kf_initialized_) { 
        state_str = "STATE2_COAST"; 
        custom_state_msg.data = 2; 
    } 
    else { 
        state_str = "STATE2_TRACK"; 
        custom_state_msg.data = 2; 
    }
    
    // Broadcast the state for the data logger
    custom_state_pub_->publish(custom_state_msg);

    // RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 500,
    //     "[STATUS] %-13s | tag_vis=%d | kf_init=%d | lost_cnt=%d | "
    //     "Alt=%.2f | Target=%.2f | pos_sp_z=%.2f",
    //     state_str, (int)tag_visible_, (int)kf_initialized_, lost_tag_count_,
    //     actual_altitude, current_target_altitude_, pos_sp_ned_.z());
    // =========================================================================

>>>>>>> a79a21d (final hardware with csv logger)
    // ---------------------------------------------------------
    //      STATE 1: SAFETY HOVER (Tag Lost or Not Initialized)
    // ---------------------------------------------------------
    if (!tag_visible_ || !kf_initialized_) 
    {
        // Read manual RC sticks for fallback control
        float roll_cmd = with_deadband(_manual_control_input_->roll(), Config::RC_DEADBAND);
        float pitch_cmd = with_deadband(_manual_control_input_->pitch(), Config::RC_DEADBAND);
        float throttle_cmd = with_deadband(_manual_control_input_->throttle(), Config::RC_DEADBAND);

        // Convert stick inputs to body-frame velocity commands
<<<<<<< HEAD
        float cmd_v_x = roll_cmd * Config::RC_MAX_XY_VEL;     // Max vel in X (Forward/Back)
        float cmd_v_y = pitch_cmd * Config::RC_MAX_XY_VEL;    // Max vel in Y (Left/Right)

        // Rotate body velocities to NED world frame based on current heading
        float yaw_now = drone_yaw_ned_;
        float cos_yaw = std::cos(yaw_now);
        float sin_yaw = std::sin(yaw_now);
        float v_n = cmd_v_x * cos_yaw - cmd_v_y * sin_yaw;
        float v_e = cmd_v_x * sin_yaw + cmd_v_y * cos_yaw;

        float v_d = -throttle_cmd * Config::RC_MAX_Z_VEL;       // Max vel in Z (Descent)

        // Integrate manual commands into the hover position setpoint
        yaw_sp_rad_ = wrap_pi(yaw_now + (_manual_control_input_->yaw() * Config::RC_MAX_YAW_RATE)*dt_s); 
        pos_sp_ned_.x() += v_n * dt_s;
        pos_sp_ned_.y() += v_e * dt_s;
        pos_sp_ned_.z() += v_d * dt_s;

        _goto_setpoint_->update(pos_sp_ned_, yaw_sp_rad_, Config::RC_MAX_XY_VEL, Config::RC_MAX_Z_VEL, Config::RC_MAX_YAW_RATE); 
=======
        Eigen::Vector2f v_body(roll_cmd * Config::RC_MAX_XY_VEL, pitch_cmd * Config::RC_MAX_XY_VEL);

        // Rotate body velocities to NED world frame based on current heading
        // v_ned = R(psi) * v_body, where R(psi) is the 2D rotation matrix for the drone's yaw
        Eigen::Vector2f v_ned = Eigen::Rotation2Df(drone_yaw_ned_) * v_body;

        float v_d = -throttle_cmd * Config::RC_MAX_Z_VEL;       // Max vel in Z (Descent)

        // // Integrate manual commands into the hover position setpoint
        // yaw_sp_rad_ = wrap_pi(drone_yaw_ned_ + (_manual_control_input_->yaw() * Config::RC_MAX_YAW_RATE)*dt_s); 
        
        // pos_sp_ned_.x() += -v_ned(0) * dt_s;
        // pos_sp_ned_.y() += v_ned(1) * dt_s;
        // pos_sp_ned_.z() += v_d * dt_s;

        // _goto_setpoint_->update(pos_sp_ned_, yaw_sp_rad_, Config::RC_MAX_XY_VEL, Config::RC_MAX_Z_VEL, Config::RC_MAX_YAW_RATE); 
        // return;

        // ==============================================================================
        // Bypass MPC and directly feed stick velocities to the PX4 Trajectory Setpoint
        sp.withHorizontalVelocity(Eigen::Vector2f(v_ned(1), v_ned(0)));
        sp.withVelocityZ(v_d);
        sp.withYawRate(_manual_control_input_->yaw() * Config::RC_MAX_YAW_RATE);

        // Keep the drone locked against the chipped prop even in manual hover, 
        // unless you are actively pushing the yaw stick.
        if (std::abs(_manual_control_input_->yaw()) < 0.05f) 
        {
            sp.withYaw(drone_yaw_ned_);
        } 
        else 
        {
            sp.withYaw(std::numeric_limits<float>::quiet_NaN());
        }

        trajectory_setpoint_->update(sp); 
>>>>>>> a79a21d (final hardware with csv logger)
        return;
    }  

    // ---------------------------------------------------------
    //      STATE 2: ACTIVE TRACKING (Tag Visible)
    // ---------------------------------------------------------
    pos_sp_ned_.z() = -current_target_altitude_;

    // Continuously sync the fallback hover position to the drone's current location 
    // so it halts exactly where it is if the tag is permanently lost.
    pos_sp_ned_.x() = drone_pos_ned_(0);
    pos_sp_ned_.y() = drone_pos_ned_(1);

    // Run Kalman Filter Absolute Position Prediction Step
    F_kf_(0, 2) = dt_s;                     // x_abs += vx * dt
    F_kf_(1, 3) = dt_s;                     // y_abs += vy * dt
    
    if (tag_visible_) 
    {
        kf_x_ = F_kf_ * kf_x_;
        kf_P_ = F_kf_ * kf_P_ * F_kf_.transpose() + Q_kf_;
    }

<<<<<<< HEAD
    // ---------------------------------------------------------
    //      STATE 2: ACTIVE TRACKING (Tag Visible)
    // ---------------------------------------------------------
    // Run Kalman Filter Prediction Step
    F_kf_(0, 2) = dt_s;                     // x += vx * dt
    F_kf_(1, 3) = dt_s;                     // y += vy * dt
    kf_x_ = F_kf_ * kf_x_;
    kf_P_ = F_kf_ * kf_P_ * F_kf_.transpose() + Q_kf_;

=======
>>>>>>> a79a21d (final hardware with csv logger)
    // Exponential Moving Average (EMA) for smoother velocity estimates
    Eigen::Vector2f raw_tag_vel(kf_x_(2), kf_x_(3));
    float alpha = Config::EMA_ALPHA;
    smoothed_tag_vel_ = (alpha * raw_tag_vel) + ((1.0f - alpha) * smoothed_tag_vel_);
    
    float vn = std::clamp(smoothed_tag_vel_(0), -max_rover_speed_, max_rover_speed_);
    float ve = std::clamp(smoothed_tag_vel_(1), -max_rover_speed_, max_rover_speed_);

    // Field of View (FOV) and Lookahead Safety Limits
    // Calculate how far ahead the drone can look without losing the tag based on altitude
    float safe_alt = std::max(0.2f, actual_altitude);
    float fov_radius = safe_alt * std::tan(Config::FOV_ANGLE_DEG * M_PI / 180.0f) * Config::FOV_SAFE_SCALE; 
    float t_lookahead = std::max(0.0f, (actual_altitude - touchdown_altitude_) * Config::LOOKAHEAD_SCALE);
    
    float offset_north = vn * t_lookahead;
    float offset_east = ve * t_lookahead;
    
    // Clamp the lead offset so the predictive target never leaves the camera FOV
    float lead_offset = std::hypot(offset_north, offset_east);
    float max_allowed_offset = fov_radius * Config::MAX_LEAD_SCALE;
    
    if (lead_offset > max_allowed_offset) 
    {
        float scale = max_allowed_offset / lead_offset;
        offset_north *= scale;
        offset_east *= scale;
    }

<<<<<<< HEAD
    // Calculate Predictive Error for the MPC
    float future_tag_north = kf_x_(0) + offset_north;
    float future_tag_east  = kf_x_(1) + offset_east;

    Eigen::Vector2f drone_in_tag_pos(drone_pos_ned_(0) - future_tag_north, drone_pos_ned_(1) - future_tag_east);
    float horiz_error = drone_in_tag_pos.norm();
=======
    // Compute Predictive Future Absolute Coordinates of the Target
    float future_tag_offset_north = kf_x_(0) + offset_north;
    float future_tag_offset_east  = kf_x_(1) + offset_east;

    // THE FIX: Standard MPC tracking error definition is explicitly (Drone - Target)
    error_vector(0) = drone_pos_ned_(0) - future_tag_offset_north;
    error_vector(1) = drone_pos_ned_(1) - future_tag_offset_east;

    horiz_error = error_vector.norm();
>>>>>>> a79a21d (final hardware with csv logger)

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

    // ---------------------------------------------------------
    //      STATE 3: DESCENT / LANDING LOGIC
    // ---------------------------------------------------------
<<<<<<< HEAD
=======
    // Ramp current_target_altitude_ toward chase_altitude_ to avoid step inputs
    if (!landing_triggered_)
    {
        float alt_error = current_target_altitude_ - chase_altitude_;
        float ramp_step = std::clamp(alt_error * 0.5f, -0.4f, 0.4f);
        current_target_altitude_ -= ramp_step * dt_s;
        current_target_altitude_ = std::max(chase_altitude_, current_target_altitude_);
    }

>>>>>>> a79a21d (final hardware with csv logger)
    float cmd_v_down = 0.0f; // Default Z-Velocity constraint (Hold Alt)

    if (landing_triggered_) 
    {
        // Only descend if the drone is horizontally aligned with the moving target
        float allowed_error = std::max(Config::LANDING_ERR_MIN, Config::LANDING_ERR_SCALE * actual_altitude);
        if (horiz_error < allowed_error) 
        {
            current_target_altitude_ -= descent_rate_ * dt_s;
            cmd_v_down = descent_rate_;     // Apply downward velocity
        }
        current_target_altitude_ = std::max(touchdown_altitude_, current_target_altitude_);

        if (actual_altitude <= (touchdown_altitude_ + Config::TOUCHDOWN_ALT_MARGIN) && horiz_error < Config::TOUCHDOWN_ERR_MAX) 
        {
            RCLCPP_WARN(node().get_logger(), "TOUCHDOWN DETECTED! Completing Mode.");
            completed(px4_ros2::Result::Success); 
            return;
        }
    }

    // ---------------------------------------------------------
    //          STATE 4: COMMAND DISPATCH (Bridging to PX4)
    // ---------------------------------------------------------
    // Send state to the external Python MPC
    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {error_vector(0), error_vector(1), vn, ve, actual_altitude};
    mpc_state_pub_->publish(state_msg);

    // Fetch the latest optimized command from the MPC
    float cmd_v_north = mpc_vel_cmd_(0); 
    float cmd_v_east  = mpc_vel_cmd_(1);

    // Yaw Control: Keep the drone facing the direction of the moving rover
    float yaw_rate_cmd = 0.0f;
    if (std::hypot(vn, ve) > Config::YAW_MOVE_THRESHOLD)     // Only yaw if the rover is actually moving
    {
        float target_yaw = atan2(kf_x_(3), kf_x_(2));
        float yaw_error = target_yaw - drone_yaw_ned_;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));          // wrap to -pi to pi
        
        // P-controller for Yaw Rate
        yaw_rate_cmd = std::clamp(Config::YAW_P_GAIN * yaw_error, -Config::YAW_RATE_MAX, Config::YAW_RATE_MAX);   
    }

<<<<<<< HEAD
    px4_ros2::TrajectorySetpoint sp;

=======
>>>>>>> a79a21d (final hardware with csv logger)
    // Always command horizontal velocity and yaw rate
    sp.withHorizontalVelocity(Eigen::Vector2f(cmd_v_north, cmd_v_east));
    sp.withYawRate(yaw_rate_cmd);

    // Explicitly lock the heading when not actively chasing a moving rover
    if (std::hypot(vn, ve) <= Config::YAW_MOVE_THRESHOLD) 
    {
        // Pad is stationary: Lock the drone's absolute heading so it fights the chipped prop
        sp.withYaw(drone_yaw_ned_); 
    } 
    else 
    {
        // Pad is moving: Allow the Yaw Rate controller to turn the nose freely
        sp.withYaw(std::numeric_limits<float>::quiet_NaN()); 
    }

    // Toggle between Z-Position (Chase) and Z-Velocity (Landing)
    if (landing_triggered_) sp.withVelocityZ(cmd_v_down);
    else sp.withPositionZ(-current_target_altitude_);

<<<<<<< HEAD
    // Send the cleanly built struct to PX4!
    // pos_sp_ned_.x() += cmd_v_north * dt_s;
    // pos_sp_ned_.y() += cmd_v_east * dt_s;
    // pos_sp_ned_.z() = -current_target_altitude_; // Always hold altitude in NED frame

    // yaw_sp_rad_ = wrap_pi(drone_yaw_ned_ + yaw_rate_cmd * dt_s); // Integrate yaw rate to get yaw setpoint
    // _goto_setpoint_->update(pos_sp_ned_, yaw_sp_rad_, max_rover_speed_, 1.0f, 0.5f); // Add yaw control to the hover setpoint
    
=======
>>>>>>> a79a21d (final hardware with csv logger)
    // Send the finalized tracking command to PX4
    trajectory_setpoint_->update(sp);

    // =========================================================================
    // SYSTEM STATE LOGGING & VERIFICATION
    // =========================================================================
    // try 
    // {
    //     // 1. MOCAP GROUND TRUTH (OptiTrack)
    //     auto tf_mocap_drone = tf_buffer_->lookupTransform("world", "x500_2_priyam", tf2::TimePointZero);
    //     auto tf_mocap_tag   = tf_buffer_->lookupTransform("world", "tag_priyam", tf2::TimePointZero);
    //     auto tf_camera_tag  = tf_buffer_->lookupTransform("x500_mono_cam_down_0/camera_link/imager", "active_landing_target", tf2::TimePointZero);

    //     // Calculate MoCap Yaw
    //     Eigen::Quaternionf q_wd(
    //         tf_mocap_drone.transform.rotation.w, tf_mocap_drone.transform.rotation.x,
    //         tf_mocap_drone.transform.rotation.y, tf_mocap_drone.transform.rotation.z
    //     );
    //     float yaw_mocap = atan2(2.0f * (q_wd.w()*q_wd.z() + q_wd.x()*q_wd.y()), 
    //                             1.0f - 2.0f * (q_wd.y()*q_wd.y() + q_wd.z()*q_wd.z()));

    //     // Convert MoCap NWU World to NED World for 1:1 Comparison
    //     float mocap_drone_n = tf_mocap_drone.transform.translation.x; 
    //     float mocap_drone_e = -tf_mocap_drone.transform.translation.y;
    //     float mocap_drone_d = -tf_mocap_drone.transform.translation.z;

    //     float mocap_tag_n = tf_mocap_tag.transform.translation.x; 
    //     float mocap_tag_e = -tf_mocap_tag.transform.translation.y;

    //     // --- UNIFIED SYSTEM STATE LOG ---
    //     RCLCPP_INFO_THROTTLE(node().get_logger(), *node().get_clock(), 500,
    //         "\n====================== [ SYSTEM STATE LOG ] ======================\n"
    //         "[1. SYSTEM ALIGNMENT (NED)]\n"
    //         "    1. MoCap Ground Truth : N: %6.3f | E: %6.3f | D: %6.3f | Yaw: %5.3f\n"
    //         "    2. Bridge (-> PX4)    : N: %6.3f | E: %6.3f | D: %6.3f | Yaw: %5.3f\n"
    //         "    3. PX4 EKF (<- PX4)   : N: %6.3f | E: %6.3f | D: %6.3f | Yaw: %5.3f\n"
    //         "    => EKF DRIFT DELTA    : dN:%6.3f | dE:%6.3f | dD:%6.3f | dYaw:%5.3f\n"
    //         "------------------------------------------------------------------\n"
    //         "[2. DRONE STATE & RAW VISION]\n"
    //         "    Actual Drone Vel      : Vn:%6.3f | Ve:%6.3f | Vd:%6.3f\n"
    //         "    Cam->Tag (FLU Body)   : X: %6.3f | Y: %6.3f | Z: %6.3f \n"
    //         "    Instant Offset (NED)  : dN:%6.3f | dE:%6.3f \n"
    //         "    MoCap Target Truth    : N: %6.3f | E: %6.3f \n"
    //         "------------------------------------------------------------------\n"
    //         "[3. KALMAN FILTER & PREDICTION]\n"
    //         "    Filtered Abs Target   : N: %6.3f | E: %6.3f \n"
    //         "    Target Velocity (EMA) : Vn:%6.3f | Ve:%6.3f \n"
    //         "    Future Target Pos     : N: %6.3f | E: %6.3f (Lookahead: %.2fs)\n"
    //         "------------------------------------------------------------------\n"
    //         "[4. MPC CONTROL OUTPUT]\n"
    //         "    Tracking Error        : dN:%6.3f | dE:%6.3f | Dist:%5.3f\n"
    //         "    Commanded Velocity    : Vn:%6.3f | Ve:%6.3f | Vd:%5.3f\n"
    //         "==================================================================",
            
    //         // 1. System Alignment 
    //         mocap_drone_n, mocap_drone_e, mocap_drone_d, yaw_mocap,
    //         bridge_pos_ned_(0), bridge_pos_ned_(1), bridge_pos_ned_(2), bridge_yaw_ned_,
    //         drone_pos_ned_(0), drone_pos_ned_(1), drone_pos_ned_(2), drone_yaw_ned_,
    //         (drone_pos_ned_(0) - mocap_drone_n), (drone_pos_ned_(1) - mocap_drone_e), 
    //         (drone_pos_ned_(2) - mocap_drone_d), (drone_yaw_ned_ - yaw_mocap),
            
    //         // 2. Drone State & Vision Pipeline
    //         drone_vel_ned_(0), drone_vel_ned_(1), drone_vel_ned_(2), // <-- ADDED ACTUAL VELOCITY
    //         tf_camera_tag.transform.translation.x, tf_camera_tag.transform.translation.y, tf_camera_tag.transform.translation.z,
    //         tag_offset_ned_.x(), tag_offset_ned_.y(),                // <-- FIXED SCOPE
    //         mocap_tag_n, mocap_tag_e,
            
    //         // 3. KF & Prediction 
    //         kf_x_(0), kf_x_(1),
    //         smoothed_tag_vel_(0), smoothed_tag_vel_(1),
    //         future_tag_offset_north, future_tag_offset_east, t_lookahead,
            
    //         // 4. MPC 
    //         error_vector(0), error_vector(1), horiz_error,
    //         cmd_v_north, cmd_v_east, (landing_triggered_ ? cmd_v_down : 0.0f)
    //     );
    // } 
    // catch (const tf2::TransformException & ex) 
    // {
    //     // Fail silently during active flight if MoCap drops out
    // }
}