#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

using std::placeholders::_1;

class AprilTagRelativePose : public rclcpp::Node
{
public:
    AprilTagRelativePose() : Node("apriltag_relative_pose"), 
                             tag_visible_(false), 
                             cached_tf_ready_(false)
    {
        // ---------------- Parameters ----------------
        this->declare_parameter<std::string>("frames.world", "map");
        this->declare_parameter<std::string>("frames.drone", "base_link");
        this->declare_parameter<double>("vision_timeout", 0.5);

        world_frame_ = this->get_parameter("frames.world").as_string();
        drone_frame_ = this->get_parameter("frames.drone").as_string();
        vision_timeout_ = this->get_parameter("vision_timeout").as_double();

        // ---------------- TF Setup ----------------
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---------------- Publishers & Subscribers ----------------
        rel_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/apriltag/relative_pose", 10);
        tag_visible_pub_ = this->create_publisher<std_msgs::msg::Bool>("/landing/tag_visible_flag", 10);
        tag_map_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/landing/tag_pose_map", 10);

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/landing_target_pose", 10, std::bind(&AprilTagRelativePose::target_pose_cb, this, _1));

        // ---------------- Timer ----------------
        last_tag_time_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagRelativePose::process_vision_data, this)); // 20Hz

        RCLCPP_INFO(this->get_logger(), "[APRILTAG_POSE] C++ Node Started at 20Hz.");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rel_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tag_visible_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string world_frame_;
    std::string drone_frame_;
    double vision_timeout_;
    
    bool tag_visible_;
    bool new_tag_msg_available_ = false;
    rclcpp::Time last_tag_time_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_target_msg_;
    
    bool cached_tf_ready_;
    geometry_msgs::msg::TransformStamped transform_drone_camera_;

    //=======================================================
    geometry_msgs::msg::PoseStamped prev_world_pose_;
    bool prev_pose_valid_ = false;
    rclcpp::Time prev_pose_time_;
    //=======================================================

    void target_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_target_msg_ = msg;
        last_tag_time_ = this->now();
        new_tag_msg_available_ = true;
        tag_visible_ = true;
    }

    void process_vision_data()
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_tag_time_).seconds();

        // 1. WATCHDOG — always runs every cycle regardless of new data
        bool currently_visible = (dt <= vision_timeout_);
        std_msgs::msg::Bool flag_msg;
        flag_msg.data = currently_visible;
        tag_visible_pub_->publish(flag_msg);

        if (!currently_visible)
        {
            if (tag_visible_) 
            {
                tag_visible_ = false;
                RCLCPP_WARN(this->get_logger(), "[VISION] Tag lost for %.2f seconds.", dt);
            }
            return;
        }

        // 2. POSE PROCESSING — only runs when a new detection arrived
        if (!new_tag_msg_available_ || !latest_target_msg_) return;
        new_tag_msg_available_ = false;
        tag_visible_ = true;

        // 3. CACHE STATIC TF (Drone -> Camera)
        if (!cached_tf_ready_) 
        {
            try 
            {
                transform_drone_camera_ = tf_buffer_->lookupTransform(
                    drone_frame_, latest_target_msg_->header.frame_id, latest_target_msg_->header.stamp);
                cached_tf_ready_ = true;
                RCLCPP_INFO(this->get_logger(), "[APRILTAG_POSE] Cached Static Drone->Camera TF.");
            } catch (const tf2::TransformException & ex) {
                return;
            }
        }

        // 4. RELATIVE MATH (Apply cached transform instantly)
        geometry_msgs::msg::PoseStamped tag_pose_drone;
        tf2::doTransform(*latest_target_msg_, tag_pose_drone, transform_drone_camera_);
        tag_pose_drone.header.stamp = latest_target_msg_->header.stamp;
        tag_pose_drone.header.frame_id = drone_frame_;
        rel_pose_pub_->publish(tag_pose_drone);

        // =================================================
        // auto &p = latest_target_msg_->pose.position;

        // RCLCPP_INFO(this->get_logger(),
        // "[RAW CAMERA FRAME]\n"
        // "X=%.3f Y=%.3f Z=%.3f\n"
        // "frame=%s",
        // p.x, p.y, p.z,
        // latest_target_msg_->header.frame_id.c_str());

        // auto &d = tag_pose_drone.pose.position;

        // RCLCPP_INFO(this->get_logger(),
        // "[DRONE FRAME]\n"
        // "X=%.3f Y=%.3f Z=%.3f\n"
        // "frame=%s",
        // d.x, d.y, d.z,
        // tag_pose_drone.header.frame_id.c_str());
        // =================================================

        // 5. MAP FRAME PUBLISH (Optional RViz Debugging)
        try 
        {
            // =================================================
            // rclcpp::Time tf_now = this->now();

            // rclcpp::Time sensor_stamp(latest_target_msg_->header.stamp);

            // double sensor_latency =
            //     (tf_now - sensor_stamp).seconds();

            // RCLCPP_WARN(this->get_logger(),
            // "[LATENCY] sensor_age = %.4f sec",
            // sensor_latency);
            // =================================================

            geometry_msgs::msg::TransformStamped transform_world_drone = tf_buffer_->lookupTransform(
                world_frame_, drone_frame_, latest_target_msg_->header.stamp);
            geometry_msgs::msg::PoseStamped tag_pose_world;
            tf2::doTransform(tag_pose_drone, tag_pose_world, transform_world_drone);
            tag_pose_world.header.frame_id = world_frame_;
            tag_map_pub_->publish(tag_pose_world);

            // =================================================
            // auto &w = tag_pose_world.pose.position;

            // RCLCPP_INFO(this->get_logger(),
            // "[WORLD FRAME]\n"
            // "X=%.3f Y=%.3f Z=%.3f",
            // w.x, w.y, w.z);

            // if (prev_pose_valid_)
            // {
            //     rclcpp::Time current_pose_time(tag_pose_world.header.stamp);
            //     double dt_motion = (current_pose_time - prev_pose_time_).seconds();

            //     if (dt_motion > 0.001)
            //     {
            //         double dx = w.x - prev_world_pose_.pose.position.x;
            //         double dy = w.y - prev_world_pose_.pose.position.y;
            //         double dz = w.z - prev_world_pose_.pose.position.z;

            //         double vx = dx / dt_motion;
            //         double vy = dy / dt_motion;
            //         double vz = dz / dt_motion;

            //         double motion_norm = std::sqrt(dx*dx + dy*dy);

            //         RCLCPP_WARN(this->get_logger(),
            //         "\n============= TAG MOTION DEBUG =============\n"
            //         "dt         : %.4f sec\n"
            //         "dx dy dz   : %.4f %.4f %.4f\n"
            //         "vx vy vz   : %.4f %.4f %.4f\n"
            //         "2D motion  : %.4f m\n"
            //         "===========================================",
            //         dt_motion,
            //         dx, dy, dz,
            //         vx, vy, vz,
            //         motion_norm);
            //     }
            // }

            // prev_world_pose_ = tag_pose_world;
            // prev_pose_time_ = rclcpp::Time(tag_pose_world.header.stamp);
            // prev_pose_valid_ = true;
            // =================================================

        } 
        catch (const tf2::TransformException & ex) {}
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagRelativePose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}