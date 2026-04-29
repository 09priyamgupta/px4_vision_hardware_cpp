#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from apriltag_msgs.msg import AprilTagDetectionArray

class LandingDirector(Node):
    def __init__(self):
        super().__init__('landing_director')
              
        # ------------------------------- TF2 Setup -------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ---------------------------- Camera Frame ID ----------------------------
        self.declare_parameter('frames.camera', 'x500_mono_cam_0/camera_link/imager_optical')
        self.camera_frame = self.get_parameter('frames.camera').value
        
        # --------------------- Switching Threshold (in meters) ---------------------
        self.declare_parameter('tag_switch_altitude', 0.60)
        self.switch_altitude = self.get_parameter('tag_switch_altitude').value

        # -------------------------------- Publisher --------------------------------
        self.pose_pub = self.create_publisher(PoseStamped, '/landing_target_pose', 10)

        # -------------------------------- Subscriber --------------------------------
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)
        
        self.get_logger().info(f"Landing Director Node Started using {self.camera_frame}. Waiting for tags...")


    # ---------------------- Helper Functions ----------------------
    def get_transform(self, target_frame):
        '''
            Helper function to look up a TF frame and convert it to a PoseStamped
        '''
        try:
            # Look up the transform from the camera to the target tag/bundle
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, 
                target_frame, 
                rclpy.time.Time())
            
            # Convert Transform to PoseStamped
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            return pose
            
        except Exception as e:
            return None
        
    
    # ---------------------- Callbacks ----------------------
    def detections_callback(self, msg: AprilTagDetectionArray):
        '''
            This callback is triggered whenever new AprilTag detections are received.
        '''
        exact_time = msg.header.stamp

        # Fetch ALL transforms for logging
        bundle_pose = self.get_transform('landing_pad')
        tag_19_pose = self.get_transform('tag36h11:19')

        active_pose = None

        # Decision Logic
        if bundle_pose is not None:

            z_distance = bundle_pose.pose.position.z
            # self.get_logger().info(f"[DEBUG] Bundle Z: {z_distance:.2f}, Switch Threshold: {self.switch_altitude}", throttle_duration_sec=1.0)
            
            # LOGIC 1: High Altitude -> Trust the Bundle
            if z_distance > self.switch_altitude:
                self.pose_pub.publish(bundle_pose)
                active_pose = bundle_pose
                active_target = "Bundle"
                
            # LOGIC 2: Low Altitude -> Switch to Small Tag (if visible)
            elif z_distance <= self.switch_altitude and tag_19_pose is not None:
                self.pose_pub.publish(tag_19_pose)
                active_pose = tag_19_pose
                active_target = "Tag 19"
                
            # LOGIC 3: Low Altitude but Small Tag not visible -> Fallback to Bundle
            else:
                self.pose_pub.publish(bundle_pose)
                active_pose = bundle_pose
                active_target = "Bundle (Fallback)"

        # LOGIC 4: Bundle lost, but Small Tag visible -> Trust Small Tag
        elif tag_19_pose is not None:
            self.pose_pub.publish(tag_19_pose)
            active_pose = tag_19_pose

        # Broadcasting the TF frame of detected AprilTag Bundle
        if active_pose is not None:
            self.pose_pub.publish(active_pose)

            t = TransformStamped()
            t.header.stamp = exact_time
            
            # Parent is the camera, Child is our new dynamic target
            t.header.frame_id = self.camera_frame
            t.child_frame_id = 'active_landing_target'
            
            t.transform.translation.x = active_pose.pose.position.x
            t.transform.translation.y = active_pose.pose.position.y
            t.transform.translation.z = active_pose.pose.position.z
            t.transform.rotation = active_pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)


# ---------------------- Main Function ----------------------
def main(args=None):
    rclpy.init(args=args)
    node = LandingDirector()
    
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()