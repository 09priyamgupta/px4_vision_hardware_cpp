#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <string>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class LandingDirector : public rclcpp::Node
{
public:
    LandingDirector() : Node("landing_director"), last_update_time_(0, 0, this->get_clock()->get_clock_type())
    {
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "apriltag_node");

        // ---------------- TF2 Setup ----------------
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ---------------- Parameters ----------------
        this->declare_parameter<std::string>("frames.camera", "x500_mono_cam_0/camera_link/imager_optical");
        camera_frame_ = this->get_parameter("frames.camera").as_string();

        this->declare_parameter<double>("tag_switch_altitude", 0.60);
        switch_altitude_ = this->get_parameter("tag_switch_altitude").as_double();

        // ---------------- Publisher & Subscriber ----------------
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/landing_target_pose", 10);
        det_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/detections", 10, std::bind(&LandingDirector::detections_callback, this, _1));

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
            {
                this->drone_alt_ = -msg->position[2]; // Convert NED Z to Positive Altitude
            });

        RCLCPP_INFO(this->get_logger(), "Landing Director (C++) Started using %s. Waiting for tags...", camera_frame_.c_str());

        // ---------------- Throttling ----------------
        last_update_time_ = this->now();
        update_rate_hz_ = 15.0;
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    std::string camera_frame_;
    double switch_altitude_;
    double drone_alt_ = 10.0;

    rclcpp::Time last_update_time_;
    double update_rate_hz_;
    bool tracking_small_tag_ = false;

    rclcpp::AsyncParametersClient::SharedPtr param_client_;

    // Helper Function to look up TF
    bool get_transform(const std::string& target_frame, geometry_msgs::msg::PoseStamped& pose_out)
    {
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                camera_frame_, target_frame, tf2::TimePointZero);
            
            pose_out.header = t.header;
            pose_out.pose.position.x = t.transform.translation.x;
            pose_out.pose.position.y = t.transform.translation.y;
            pose_out.pose.position.z = t.transform.translation.z;
            pose_out.pose.orientation = t.transform.rotation;
            return true;
        } catch (const tf2::TransformException & ex) {
            return false;
        }
    }

    // Main Callback
    void detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        double age = (this->now() - msg->header.stamp).seconds();
        if (age > 0.3) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Stale Detection! Age: %.3f s", age);
        }

        // Don't process empty detection arrays 
        if (msg->detections.empty()) return;
        
        // THROTTLE: Skip processing if exceeding target Hz
        rclcpp::Time now = this->now();
        if ((now - last_update_time_).seconds() < (1.0 / update_rate_hz_)) 
        {
            return;
        }
        last_update_time_ = now;

        geometry_msgs::msg::PoseStamped bundle_pose;
        geometry_msgs::msg::PoseStamped tag_19_pose;

        bool has_bundle = get_transform("landing_pad", bundle_pose);
        bool has_tag_19 = get_transform("tag36h11:19", tag_19_pose);

        geometry_msgs::msg::PoseStamped active_pose;
        bool has_active_target = false;
        std::string active_target_name = "None";

        // Dynamic Decimation Logic
        static bool decimation_switched = false;
        if (drone_alt_ <= switch_altitude_ && !decimation_switched) 
        {
            RCLCPP_INFO(this->get_logger(), "Descending: Switching decimate to 1.0 for higher precision.");
            
            // Send async request to change parameter
            param_client_->set_parameters({
                rclcpp::Parameter("detector.decimate", 1.0)
            });
            
            decimation_switched = true; 
        }
        // Optional: Reset if we climb back up
        else if (drone_alt_ > (switch_altitude_ + 0.5) && decimation_switched)
        {
            param_client_->set_parameters({
                rclcpp::Parameter("detector.decimate", 2.0)
            });
            decimation_switched = false;
        }

        // Decision Logic
        if (has_bundle) 
        {
            // Decision Logic
            bool target_switched = false;
            
            // Always prefer Tag 19 if we are close and it's visible
            if (drone_alt_ <= switch_altitude_ && has_tag_19) 
            {
                if (!tracking_small_tag_) 
                {
                    tracking_small_tag_ = true;
                    target_switched = true;
                }
            } 
            // Only revert to bundle if we are significantly higher
            else if (drone_alt_ > (switch_altitude_ + 0.2)) 
            {
                tracking_small_tag_ = false;
            }

            if (tracking_small_tag_ && has_tag_19) 
            {
                active_pose = tag_19_pose;
                active_target_name = "Tag 19";
                has_active_target = true;
            } 
            else if (has_bundle) 
            {
                active_pose = bundle_pose;
                active_target_name = "Bundle";
                has_active_target = true;
            }
        } 
        else if (has_tag_19 && tag_19_pose.pose.position.z <= (switch_altitude_ + 0.2)) 
        {
            active_pose = tag_19_pose;
            active_target_name = "Tag 19 (Bundle Lost)";
            has_active_target = true;
            tracking_small_tag_ = true;
        }

        // ----------------------------------------------------
        // 2. GEOMETRIC OFFSET CORRECTION
        // ----------------------------------------------------
        // If we are tracking Tag 19, shift its pose back to the bundle's true center!
        if (has_active_target && tracking_small_tag_) 
        {
            try 
            {
                // Dynamically get the transform from Tag19 to Bundle Center
                geometry_msgs::msg::TransformStamped tf_tag_to_center = 
                    tf_buffer_->lookupTransform("landing_pad", "tag36h11:19", msg->header.stamp);

                tf2::Transform tf_tag19_to_center;
                tf2::fromMsg(tf_tag_to_center.transform, tf_tag19_to_center);

                // Convert active_pose to tf2::Transform
                tf2::Transform tf_cam_to_tag19;
                tf2::fromMsg(active_pose.pose, tf_cam_to_tag19);

                // Chain: Camera -> Tag19 -> Bundle Center
                tf2::Transform tf_cam_to_center = tf_cam_to_tag19 * tf_tag19_to_center;

                // Update active_pose
                active_pose.pose.position.x = tf_cam_to_center.getOrigin().x();
                active_pose.pose.position.y = tf_cam_to_center.getOrigin().y();
                active_pose.pose.position.z = tf_cam_to_center.getOrigin().z();
                
                auto q = tf_cam_to_center.getRotation();
                active_pose.pose.orientation.x = q.x();
                active_pose.pose.orientation.y = q.y();
                active_pose.pose.orientation.z = q.z();
                active_pose.pose.orientation.w = q.w();
                
                active_target_name += " (Dynamic Offset Applied)";
            } 
            catch (const tf2::TransformException & ex) 
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dynamic Offset TF failed: %s", ex.what());
            }
        }

        // Broadcasting the TF frame
        if (has_active_target) 
        {
            pose_pub_->publish(active_pose);

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = msg->header.stamp;
            t.header.frame_id = camera_frame_;
            t.child_frame_id = "active_landing_target";

            t.transform.translation.x = active_pose.pose.position.x;
            t.transform.translation.y = active_pose.pose.position.y;
            t.transform.translation.z = active_pose.pose.position.z;
            t.transform.rotation = active_pose.pose.orientation;

            auto q = t.transform.rotation;
            if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Ignoring NaN quaternion from %s.", active_target_name.c_str());
                return;
            }

            if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Fixing zero quaternion from %s.", active_target_name.c_str());
                t.transform.rotation.w = 1.0;
            }

            tf_broadcaster_->sendTransform(t);

            // Debugging the Switching Logic
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Target: %s | Lock: %s | HasBundle: %d | HasTag19: %d | Alt: %.2f", 
                active_target_name.c_str(), 
                tracking_small_tag_ ? "TRUE" : "FALSE",
                (int)has_bundle, (int)has_tag_19, drone_alt_);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandingDirector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}