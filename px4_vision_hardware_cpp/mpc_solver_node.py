#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from px4_vision_hardware_cpp.utils.mpc_tracker import MPCTracker


class MpcSolverNode(Node):
    def __init__(self):
        super().__init__('mpc_solver_node')

        # Load parameters
        self.declare_parameter('mpc.horizon', 20)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('target_altitude', 4.0)
        self.declare_parameter('frames.world', 'map')

        horizon = self.get_parameter('mpc.horizon').value
        dt = 1.0 / self.get_parameter('control_rate').value
        altitude = self.get_parameter('target_altitude').value
        self.world_frame = self.get_parameter('frames.world').value

        # Initialize YOUR exact MPC class
        self.mpc = MPCTracker(horizon=horizon, dt=dt, fov_deg=40.0, altitude=altitude)

        # ROS 2 Bridge Pub/Sub
        self.state_sub = self.create_subscription(Float32MultiArray, '/mpc/state', self.state_callback, 10)
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/mpc/command', 10)
        self.drone_path_pub = self.create_publisher(Path, '/mpc/predicted_path', 10)
        self.rover_path_pub = self.create_publisher(Path, '/mpc/rover_predicted_path', 10)

        self.get_logger().info("[MPC PYTHON] Solver Bridge Active. Waiting for C++ Mode...")


    # ------------------------------------- Callback ---------------------------------------
    def state_callback(self, msg):
        # Extract the array sent by C++: [pos_n, pos_e, tag_vn, tag_ve, alt]
        if len(msg.data) < 5:
            return
            
        current_pos = np.array([msg.data[0], msg.data[1]])
        tag_vel = np.array([msg.data[2], msg.data[3]])
        actual_altitude = msg.data[4]

        # Call your cvxpy solver!
        vel_cmd = self.mpc.solve(current_pos, tag_vel, actual_altitude)

        self.get_logger().info(f"[MPC] Solved -> Vn: {vel_cmd[0]:.2f}, Ve: {vel_cmd[1]:.2f}", throttle_duration_sec=0.5)

        # Send the answer back to C++
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(vel_cmd[0]), float(vel_cmd[1])]
        self.cmd_pub.publish(cmd_msg)

        # --- Publish Predicted Paths ---
        try:
            drone_pred, rover_pred = self.mpc.get_predictions()
            
            # Helper function to build Path messages
            def build_path(points, frame_id="map"):
                path = Path()
                path.header.stamp = self.get_clock().now().to_msg()
                path.header.frame_id = frame_id
                for pt in points:
                    pose = PoseStamped()
                    pose.pose.position.x = float(pt[0])
                    pose.pose.position.y = float(pt[1])
                    pose.pose.position.z = float(actual_altitude) # Keep flat for visualization
                    path.poses.append(pose)
                return path

            self.drone_path_pub.publish(build_path(drone_pred, frame_id=self.world_frame))
            self.rover_path_pub.publish(build_path(rover_pred, frame_id=self.world_frame))
        except AttributeError:
            self.get_logger().warn("mpc.get_predictions() not found or failed.", once=True)

        
# ----------------- MPC Solver Node ----------------
def main():
    rclpy.init()
    node = MpcSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()