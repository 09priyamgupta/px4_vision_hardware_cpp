#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Float32

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_matrix, euler_from_quaternion

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_vision_hardware_cpp.utils.common import wrap_angle


class RoverRelativeStateNode(Node):
    '''
        Computes the relative state of the rover w.r.t the drone.

        Inputs:
        - Rover state from EKF
        - Drone state from PX4

        Outputs:
        - Relative pose (drone body frame)
        - Relative velocity (drone body frame)
        - Relative yaw
    '''
    def __init__(self):
        super().__init__('rover_relative_state_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.drone', 'base_link')
        self.declare_parameter('frames.rover_odom', 'r1_rover/odom')
        self.declare_parameter('frames.rover_relative', 'rover_relative')

        self.world_frame = self.get_parameter('frames.world').value
        self.drone_frame = self.get_parameter('frames.drone').value
        self.rover_odom_frame = self.get_parameter('frames.rover_odom').value
        self.rover_relative_frame = self.get_parameter('frames.rover_relative').value

        self.get_logger().info(f"[FRAMES] world={self.world_frame}, drone={self.drone_frame}, rover_odom={self.rover_odom_frame}, rover_relative={self.rover_relative_frame}")

        # ---------------- State variables ----------------
        self.rover_pos_world = None
        self.rover_vel_world = None
        self.rover_yaw_world = None      

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- QoS Setup ----------------
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscribers ----------------
        self.create_subscription(Odometry, '/rover/ekf_odom', self.rover_state_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.drone_odom_callback, qos_profile)

        # ---------------- Publishers ----------------
        self.drone_enu_pub = self.create_publisher(PoseStamped, '/debug/drone_pose_enu', qos_profile)
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/rover/relative_pose', qos_profile)
        self.relative_yaw_pub = self.create_publisher(Float32, '/rover/relative_yaw', qos_profile)
        self.relative_vel_pub = self.create_publisher(TwistStamped, '/rover/relative_velocity', qos_profile)

        self.get_logger().info('[ROVER_RELATIVE] Rover Relative State Node has been started.')


    # ----------------- Callbacks ----------------
    def rover_state_callback(self, msg: Odometry):
        '''
            Callback for rover state from EKF. 
        '''
        self.rover_pos_world = np.array([
                                            msg.pose.pose.position.x,    
                                            msg.pose.pose.position.y,
                                            msg.pose.pose.position.z
                                        ])
        
        q = msg.pose.pose.orientation
        _, _, self.rover_yaw_world = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.rover_vel_world = np.array([
                                            msg.twist.twist.linear.x,
                                            msg.twist.twist.linear.y,
                                            msg.twist.twist.linear.z
                                        ])       
        
    def drone_odom_callback(self, msg: VehicleOdometry):
        '''
            Callback for drone odometry (PX4 native NED frame). 
            We must convert NED to ENU to match the Rover.

            NED (North-East-Down) -> ENU (East-North-Up)
                X_enu = Y_ned
                Y_enu = X_ned
                Z_enu = -Z_ned
        '''        
        # Position Conversion
        drone_pos_world = np.array([
                                            msg.position[1],   # East
                                            msg.position[0],   # North
                                            -msg.position[2]   # Up
                                        ])

        # Velocity Conversion
        drone_vel_world = np.array([
                                            msg.velocity[1],   # East
                                            msg.velocity[0],   # North
                                            -msg.velocity[2]   # Up
                                        ])

        # Yaw Conversion (NED to ENU)
        q_ned = msg.q
        q_enu = [q_ned[1], q_ned[0], -q_ned[2], q_ned[3]]

        _, _, yaw_enu = euler_from_quaternion(q_enu)
        drone_yaw_world = wrap_angle(yaw_enu)

        now = self.get_clock().now().to_msg()

        # --- Publish Drone ENU Pose ---
        drone_pose_msg = PoseStamped()
        drone_pose_msg.header.stamp = now
        drone_pose_msg.header.frame_id = self.world_frame
        drone_pose_msg.pose.position.x = float(drone_pos_world[0])
        drone_pose_msg.pose.position.y = float(drone_pos_world[1])
        drone_pose_msg.pose.position.z = float(drone_pos_world[2])
        drone_pose_msg.pose.orientation.x = float(q_enu[0])
        drone_pose_msg.pose.orientation.y = float(q_enu[1])
        drone_pose_msg.pose.orientation.z = float(q_enu[2])
        drone_pose_msg.pose.orientation.w = float(q_enu[3])
        self.drone_enu_pub.publish(drone_pose_msg)

        # Broadcast Drone's World Pose to TF
        t_drone = TransformStamped()
        t_drone.header.stamp = now
        t_drone.header.frame_id = self.world_frame
        t_drone.child_frame_id = self.drone_frame
        t_drone.transform.translation.x = float(drone_pos_world[0])
        t_drone.transform.translation.y = float(drone_pos_world[1])
        t_drone.transform.translation.z = float(drone_pos_world[2])
        t_drone.transform.rotation.x = float(q_enu[0])
        t_drone.transform.rotation.y = float(q_enu[1])
        t_drone.transform.rotation.z = float(q_enu[2])
        t_drone.transform.rotation.w = float(q_enu[3])
        self.tf_broadcaster.sendTransform(t_drone)

        # ------------- RELATIVE MATH (Event-Triggered) -------------

        # Check if we have received at least one rover message yet
        if self.rover_pos_world is None:
            return
        
        try:
            tf_world_drone = self.tf_buffer.lookup_transform(self.drone_frame, self.world_frame, rclpy.time.Time())
        except Exception as e:
            return
        
        q = tf_world_drone.transform.rotation
        R_world_drone = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

        # Compute Relative Pose
        p_rel_world = self.rover_pos_world - drone_pos_world
        p_rel_drone = R_world_drone @ p_rel_world

        # Compute Relative Velocity
        v_rel_world = self.rover_vel_world - drone_vel_world
        v_rel_drone = R_world_drone @ v_rel_world

        # Compute Relative Yaw
        yaw_relative = wrap_angle(self.rover_yaw_world - drone_yaw_world)

        # Publish Pose (Rover relative to drone)
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.drone_frame
        pose.pose.position.x = p_rel_drone[0]
        pose.pose.position.y = p_rel_drone[1]
        pose.pose.position.z = p_rel_drone[2]
        pose.pose.orientation.w = 1.0
        self.relative_pose_pub.publish(pose)

        # Publish Velocity (Rover relative to drone)
        vel = TwistStamped()
        vel.header = pose.header
        vel.twist.linear.x = v_rel_drone[0]
        vel.twist.linear.y = v_rel_drone[1]
        vel.twist.linear.z = v_rel_drone[2]
        self.relative_vel_pub.publish(vel)

        # Publish Relative Yaw
        yaw_msg = Float32()
        yaw_msg.data = yaw_relative
        self.relative_yaw_pub.publish(yaw_msg)

        # Broadcast Relative TF
        tf_out = TransformStamped()
        tf_out.header = pose.header
        tf_out.child_frame_id = self.rover_relative_frame
        tf_out.transform.translation.x = p_rel_drone[0]
        tf_out.transform.translation.y = p_rel_drone[1]
        tf_out.transform.translation.z = p_rel_drone[2]
        tf_out.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_out)
        

# ------------------ Main Function ----------------
def main():
    rclpy.init()
    node = RoverRelativeStateNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()