#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray


class AprilTagRelativePose(Node):
    '''
        This node computes the relative pose of the drone (base_link) with respect
        to the AprilTag frame using TF.
        It uses the drone's altitude to dynamically switch which tag ID it focuses on.
    '''
    def __init__(self):
        super().__init__('apriltag_relative_pose')

        # ---------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.tag', 'landing_pad') # Defaults to bundle name
        self.declare_parameter('frames.drone', 'base_link')

        self.world_frame = self.get_parameter('frames.world').value
        self.tag_frame = self.get_parameter('frames.tag').value
        self.drone_frame = self.get_parameter('frames.drone').value

        self.get_logger().info(f"[FRAMES] world={self.world_frame}, drone={self.drone_frame}, tag={self.tag_frame}")

        # ---------------- State storage ----------------
        self.tag_visible = False
        self.last_tag_detection_time = self.get_clock().now()

        # Get the timeouts and IDs from YAML
        self.declare_parameter('bundle_ids', [19, 11, 23])
        self.bundle_ids = self.get_parameter('bundle_ids').value

        self.declare_parameter('vision_timeout', 0.5)             
        self.vision_timeout = self.get_parameter('vision_timeout').value

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Publisher ----------------
        self.rel_pose_pub = self.create_publisher(PoseStamped, '/apriltag/relative_pose', qos)
        self.tag_visible_pub = self.create_publisher(Bool, '/landing/tag_visible_flag', qos)
        self.tag_map_pub = self.create_publisher(PoseStamped, '/landing/tag_pose_map', 10)

        # ---------------- Subscription ----------------
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_detections_callback, qos)

        # ---------------- Timer ----------------
        self.declare_parameter('perception_rate', 30.0)
        perception_rate = self.get_parameter('perception_rate').value
        dt = 1.0 / perception_rate

        self.timer = self.create_timer(dt, self.update)
        self.get_logger().info('AprilTag Relative Pose Node has been started.')

    
    # ---------------- Callbacks ----------------
    def tag_detections_callback(self, msg: AprilTagDetectionArray):
        ''' 
            If any tag from our bundle is seen, the bundle is considered visible.
        '''
        found_ids = [det.id for det in msg.detections]

        # Find exactly which tags from our bundle are currently visible
        visible_bundle_tags = [tag_id for tag_id in found_ids if tag_id in self.bundle_ids]
        
        # Check if any of our bundle IDs intersect with the found IDs
        if any(tag_id in found_ids for tag_id in self.bundle_ids):
            self.tag_visible = True
            self.last_tag_detection_time = self.get_clock().now()

            # self.get_logger().info(f"--- [VISION] Visible Bundle Tags: {visible_bundle_tags} ---", throttle_duration_sec=1.0)

        else:
            self.tag_visible = False

    
    # ----------------- Helper Functions ----------------     
    def publish_tag_in_map(self):
        '''
            Publish the AprilTag pose in the MAP frame as a PoseStamped message.
        '''
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.tag_frame, rclpy.time.Time())

            t = transform.transform.translation

            msg = PoseStamped()
            msg.header.stamp = transform.header.stamp
            msg.header.frame_id = self.world_frame

            msg.pose.position.x = t.x
            msg.pose.position.y = t.y
            msg.pose.position.z = t.z

            self.tag_map_pub.publish(msg)

        except Exception as e:
            pass        

    
    # ----------------- Update Loop -----------------
    def update(self):
        '''
            Lookup the transform:
                apriltag --> base_link
            
            and publish it as a PoseStamped message.

            Transform from apriltag frame to base_link frame gives the pose of
            the drone relative to the AprilTag.
        '''
        # Publish tag visibility
        tag_visible_msg = Bool()
        tag_visible_msg.data = self.tag_visible
        self.tag_visible_pub.publish(tag_visible_msg)

        if not self.tf_buffer.can_transform(self.tag_frame, self.drone_frame, rclpy.time.Time()):
            self.get_logger().debug(f"TF Wait: {self.tag_frame} to {self.drone_frame} not ready", throttle_duration_sec=1.0)
            return
        
        # If no detections for vision_timeout seconds, consider tag not visible/lost
        dt = (self.get_clock().now() - self.last_tag_detection_time).nanoseconds * 1e-9
        
        if dt > self.vision_timeout:   # <--- REPLACED 1.0 HERE
            self.tag_visible = False
            self.get_logger().warn(f"[VISION] Tag lost for {dt:.2f} seconds.", throttle_duration_sec=1.0)
            return
        
        try:
            # Look up transform: where is the tag relative to the drone?
            # Target: base_link (Drone), Source: tag frame
            transform = self.tf_buffer.lookup_transform(self.drone_frame, self.tag_frame, rclpy.time.Time())

            # Convert TransformStamped to PoseStamped
            tag_pose_msg = PoseStamped()
            tag_pose_msg.header.stamp = self.get_clock().now().to_msg()
            tag_pose_msg.header.frame_id = self.drone_frame

            tag_pose_msg.pose.position.x = transform.transform.translation.x
            tag_pose_msg.pose.position.y = transform.transform.translation.y
            tag_pose_msg.pose.position.z = transform.transform.translation.z

            tag_pose_msg.pose.orientation = transform.transform.rotation

            # Publish the relative pose
            self.rel_pose_pub.publish(tag_pose_msg)

            # Also publish the tag pose in map frame
            self.publish_tag_in_map()

        except Exception as e:
            self.get_logger().warn(f'Could not transform {self.tag_frame} to {self.drone_frame}: {e}')
            return


# ------------------ Main Function ----------------
def main():
    rclpy.init()
    node = AprilTagRelativePose()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()