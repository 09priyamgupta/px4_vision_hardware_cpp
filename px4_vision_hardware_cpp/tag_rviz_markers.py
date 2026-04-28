#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener


class TagRVizMarkers(Node):
    def __init__(self):
        super().__init__('tag_rviz_markers')

        # ---------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.tag', 'landing_pad')
        self.declare_parameter('frames.drone', 'base_link')

        self.world_frame = self.get_parameter('frames.world').value
        self.tag_frame = self.get_parameter('frames.tag').value
        self.drone_frame = self.get_parameter('frames.drone').value

        self.get_logger().info(f"[FRAMES] world={self.world_frame}, tag={self.tag_frame}, drone={self.drone_frame}")

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Publisher ----------------
        self.marker_pub = self.create_publisher(Marker, '/tag_debug_markers', 10)

        # ----------------- Trajectory Storage -----------------
        self.drone_path = []
        self.max_path_length = 2000
        
        # ---------------- Timer ----------------
        self.declare_parameter('control_rate', 20.0)
        control_rate = self.get_parameter('control_rate').value
        dt = 1.0 / control_rate

        self.timer = self.create_timer(dt, self.update)
        self.get_logger().info('AprilTag RViz marker node started')


    def update(self):
        now = rclpy.time.Time()

        # ----------------------- Drone position marker -----------------------
        try:
            est_tf = self.tf_buffer.lookup_transform(self.world_frame, self.drone_frame, now)

        except Exception:
            return

        drone_marker = Marker()
        drone_marker.header.frame_id = self.world_frame
        drone_marker.header.stamp = self.get_clock().now().to_msg()
        drone_marker.ns = 'drone_from_tag'
        drone_marker.id = 1
        drone_marker.type = Marker.SPHERE
        drone_marker.action = Marker.ADD

        drone_marker.pose.position.x = est_tf.transform.translation.x
        drone_marker.pose.position.y = est_tf.transform.translation.y
        drone_marker.pose.position.z = est_tf.transform.translation.z

        drone_marker.pose.orientation = est_tf.transform.rotation

        drone_marker.scale.x = 0.25
        drone_marker.scale.y = 0.25
        drone_marker.scale.z = 0.1

        drone_marker.color.r = 1.0
        drone_marker.color.g = 0.0
        drone_marker.color.b = 0.0
        drone_marker.color.a = 1.0

        drone_marker.lifetime.sec = 0
        drone_marker.lifetime.nanosec = int(0.2 * 1e9)

        # ----------------------- Drone Trajectory Path -----------------------
        p = Point()
        p.x = est_tf.transform.translation.x
        p.y = est_tf.transform.translation.y
        p.z = est_tf.transform.translation.z

        self.drone_path.append(p)

        if len(self.drone_path) > self.max_path_length:
            self.drone_path.pop(0)

        traj_marker = Marker()
        traj_marker.header.frame_id = self.world_frame
        traj_marker.header.stamp = self.get_clock().now().to_msg()
        traj_marker.ns = 'drone_trajectory'
        traj_marker.id = 2
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD

        traj_marker.scale.x = 0.05  # line width

        traj_marker.color.r = 0.0
        traj_marker.color.g = 0.0
        traj_marker.color.b = 1.0
        traj_marker.color.a = 1.0

        traj_marker.points = self.drone_path

        # ---------------------- AprilTag Marker ------------------------
        try:
            tag_tf = self.tf_buffer.lookup_transform(self.world_frame, self.tag_frame, now)

            tag_marker = Marker()
            tag_marker.header.frame_id = self.world_frame
            tag_marker.header.stamp = self.get_clock().now().to_msg()
            tag_marker.ns = 'apriltag'
            tag_marker.id = 0
            tag_marker.type = Marker.CUBE
            tag_marker.action = Marker.ADD

            # Position (Point)
            tag_marker.pose.position.x = tag_tf.transform.translation.x
            tag_marker.pose.position.y = tag_tf.transform.translation.y
            tag_marker.pose.position.z = tag_tf.transform.translation.z

            # Orientation (Quaternion)
            tag_marker.pose.orientation = tag_tf.transform.rotation

            tag_marker.scale.x = 0.3
            tag_marker.scale.y = 0.3
            tag_marker.scale.z = 0.01

            tag_marker.color.r = 1.0
            tag_marker.color.g = 1.0
            tag_marker.color.b = 0.0
            tag_marker.color.a = 1.0

            tag_marker.lifetime.sec = 0
            tag_marker.lifetime.nanosec = int(0.2 * 1e9)

            self.marker_pub.publish(tag_marker)

        except Exception:
            return

        # ---------------- Publish -----------------
        self.marker_pub.publish(drone_marker)
        self.marker_pub.publish(traj_marker)


# ------------------- Main Function -------------------
def main():
    rclpy.init()
    node = TagRVizMarkers()

    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
