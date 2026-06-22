#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <cmath>

// ROS 2 Messages
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Raw Vision Topic
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

class LandingDataLogger : public rclcpp::Node
{
public:
    LandingDataLogger() : Node("landing_data_logger")
    {
        // ---------------- TF2 Setup ----------------
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---------------- Create CSV File ----------------
        std::string timestamp = std::to_string(static_cast<int>(this->now().seconds()));
        std::string filename = "hardware_analysis/thesis_landing_log_" + timestamp + ".csv";
        csv_file_.open(filename);

        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
            return;
        }

        // Write CSV Header (Added raw vision, drone velocities, and yaw)
        csv_file_ << "timestamp,"
                  << "mocap_drone_n,mocap_drone_e,mocap_drone_d,"
                  << "mocap_pad_n,mocap_pad_e,"
                  << "px4_drone_n,px4_drone_e,px4_drone_d,"
                  << "px4_vel_n,px4_vel_e,px4_vel_d,px4_yaw,"
                  << "raw_cam_tag_x,raw_cam_tag_y,raw_cam_tag_z,"
                  << "kf_target_n,kf_target_e,kf_vel_n,kf_vel_e,"
                  << "ema_vel_n,ema_vel_e,"
                  << "mpc_error_n,mpc_error_e,total_horiz_error,"
                  << "mpc_cmd_vn,mpc_cmd_ve,"
                  << "raw_bundle_visible,raw_tag19_visible,"
                  << "tag_visible,using_small_tag,landing_funnel_limit,"
                  << "px4_nav_state,custom_mission_state\n"; 

        // ---------------- Subscriptions ----------------
        auto qos = rclcpp::SensorDataQoS();

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&LandingDataLogger::odom_cb, this, _1));

        px4_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos, std::bind(&LandingDataLogger::px4_status_cb, this, _1));

        custom_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/landing/custom_mission_state", qos, std::bind(&LandingDataLogger::custom_state_cb, this, _1));

        tag_vis_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/landing/tag_visible_flag", qos, std::bind(&LandingDataLogger::tag_vis_cb, this, _1));

        // NEW: Subscribe to raw detections for pure optical performance
        raw_det_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/detections", 10, std::bind(&LandingDataLogger::raw_det_cb, this, _1));

        raw_tag_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/apriltag/relative_pose", qos, std::bind(&LandingDataLogger::raw_tag_cb, this, _1));

        mpc_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mpc/state", qos, std::bind(&LandingDataLogger::mpc_state_cb, this, _1));

        mpc_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mpc/command", qos, std::bind(&LandingDataLogger::mpc_cmd_cb, this, _1));

        kf_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mpc/debug/kf_state", qos, std::bind(&LandingDataLogger::kf_state_cb, this, _1));

        ema_vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mpc/debug/ema_vel", qos, std::bind(&LandingDataLogger::ema_vel_cb, this, _1));

        horiz_error_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/mpc/debug/errors", qos, std::bind(&LandingDataLogger::error_cb, this, _1));

        // ---------------- Logging Timer (30 Hz) ----------------
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&LandingDataLogger::log_data, this));

        RCLCPP_INFO(this->get_logger(), "Logger Started! Writing to: %s", filename.c_str());
    }

    ~LandingDataLogger()
    {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV File closed successfully.");
        }
    }

private:
    std::ofstream csv_file_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr custom_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_vis_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr raw_det_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_tag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr kf_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ema_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr horiz_error_sub_;

    // State Variables
    float px4_n_ = 0.0, px4_e_ = 0.0, px4_d_ = 0.0;
    float px4_vn_ = 0.0, px4_ve_ = 0.0, px4_vd_ = 0.0, px4_yaw_ = 0.0;
    float raw_tag_x_ = 0.0, raw_tag_y_ = 0.0, raw_tag_z_ = 0.0;
    float kf_n_ = 0.0, kf_e_ = 0.0, kf_vn_ = 0.0, kf_ve_ = 0.0;
    float ema_vn_ = 0.0, ema_ve_ = 0.0;
    float err_n_ = 0.0, err_e_ = 0.0, total_err_ = 0.0;
    float cmd_vn_ = 0.0, cmd_ve_ = 0.0;
    float drone_alt_ = 0.0;
    int tag_visible_ = 0;
    int raw_bundle_visible_ = 0;
    int raw_tag19_visible_ = 0;
    uint8_t px4_nav_state_ = 0;
    int32_t custom_mission_state_ = -1;

    // Callbacks
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        px4_n_ = msg->position[0]; px4_e_ = msg->position[1]; px4_d_ = msg->position[2];
        px4_vn_ = msg->velocity[0]; px4_ve_ = msg->velocity[1]; px4_vd_ = msg->velocity[2];
        drone_alt_ = -px4_d_;

        // Fast Quaternion to Yaw
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        px4_yaw_ = atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
    }
    
    void px4_status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) { px4_nav_state_ = msg->nav_state; }
    void custom_state_cb(const std_msgs::msg::Int32::SharedPtr msg) { custom_mission_state_ = msg->data; }
    void raw_tag_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { raw_tag_x_ = msg->pose.position.x; raw_tag_y_ = msg->pose.position.y; raw_tag_z_ = msg->pose.position.z; }
    void tag_vis_cb(const std_msgs::msg::Bool::SharedPtr msg) { tag_visible_ = msg->data ? 1 : 0; }
    
    void raw_det_cb(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
    {
        raw_bundle_visible_ = 0;
        raw_tag19_visible_ = 0;

        for (const auto& det : msg->detections) 
        {
            int tag_id = det.id; // Directly extract the integer ID

            if (tag_id == 19) 
            {
                raw_tag19_visible_ = 1;
            } 
            else 
            {
                raw_bundle_visible_ = 1; // Any other ID is the outer bundle
            }
        }
    }

    void mpc_state_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { if (msg->data.size() >= 2) { err_n_ = msg->data[0]; err_e_ = msg->data[1]; } }
    void mpc_cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { if (msg->data.size() >= 2) { cmd_vn_ = msg->data[0]; cmd_ve_ = msg->data[1]; } }
    void kf_state_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { if (msg->data.size() >= 4) { kf_n_ = msg->data[0]; kf_e_ = msg->data[1]; kf_vn_ = msg->data[2]; kf_ve_ = msg->data[3]; } }
    void ema_vel_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { if (msg->data.size() >= 2) { ema_vn_ = msg->data[0]; ema_ve_ = msg->data[1]; } }
    void error_cb(const std_msgs::msg::Float32::SharedPtr msg) { total_err_ = msg->data; }

    // Main Logging Loop
    void log_data()
    {
        double timestamp = this->now().seconds();

        // 1. Fetch MoCap Data via TF2
        float mocap_drone_n = 0.0, mocap_drone_e = 0.0, mocap_drone_d = 0.0;
        float mocap_pad_n = 0.0, mocap_pad_e = 0.0;

        try {
            auto tf_drone = tf_buffer_->lookupTransform("world", "x500_2_priyam", tf2::TimePointZero);
            mocap_drone_n = tf_drone.transform.translation.x;
            mocap_drone_e = -tf_drone.transform.translation.y;
            mocap_drone_d = -tf_drone.transform.translation.z;
        } catch (...) { }

        try {
            auto tf_pad = tf_buffer_->lookupTransform("world", "landing_pad", tf2::TimePointZero);
            mocap_pad_n = tf_pad.transform.translation.x;
            mocap_pad_e = -tf_pad.transform.translation.y;
        } catch (...) { }

        // 2. Deduce Active Target Flag
        int using_small_tag = 0;
        if (tag_visible_ && drone_alt_ <= 0.65) {
            using_small_tag = 1;
        }

        // 3. Calculate Landing Funnel Limit 
        float landing_err_min = 0.35f;
        float landing_err_scale = 0.5f;
        float landing_funnel = std::max(landing_err_min, landing_err_scale * std::max(0.0f, drone_alt_));

        // 4. Write to CSV
        csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                  << std::setprecision(4)
                  << mocap_drone_n << "," << mocap_drone_e << "," << mocap_drone_d << ","
                  << mocap_pad_n << "," << mocap_pad_e << ","
                  << px4_n_ << "," << px4_e_ << "," << px4_d_ << ","
                  << px4_vn_ << "," << px4_ve_ << "," << px4_vd_ << "," << px4_yaw_ << ","
                  << raw_tag_x_ << "," << raw_tag_y_ << "," << raw_tag_z_ << ","
                  << kf_n_ << "," << kf_e_ << "," << kf_vn_ << "," << kf_ve_ << ","
                  << ema_vn_ << "," << ema_ve_ << ","
                  << err_n_ << "," << err_e_ << "," << total_err_ << ","
                  << cmd_vn_ << "," << cmd_ve_ << ","
                  << raw_bundle_visible_ << "," << raw_tag19_visible_ << ","
                  << tag_visible_ << "," << using_small_tag << "," << landing_funnel << ","
                  << static_cast<int>(px4_nav_state_) << "," << custom_mission_state_ << "\n";

        csv_file_.flush(); 
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandingDataLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}