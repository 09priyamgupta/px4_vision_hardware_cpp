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
from tf_transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler

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
        # self.declare_parameter('frames.rover_odom', 'r1_rover/odom')
        self.declare_parameter('frames.rover_relative', 'rover_relative')

        self.world_frame = self.get_parameter('frames.world').get_parameter_value().string_value or 'map'
        self.drone_frame = self.get_parameter('frames.drone').get_parameter_value().string_value or 'base_link'
        # self.rover_odom_frame = self.get_parameter('frames.rover_odom').value
        self.rover_relative_frame = self.get_parameter('frames.rover_relative').get_parameter_value().string_value or 'rover_relative'

        # self.get_logger().info(f"[FRAMES] world={self.world_frame}, drone={self.drone_frame}, rover_odom={self.rover_odom_frame}, rover_relative={self.rover_relative_frame}")

        # ---------------- State variables ----------------
        self.rover_pos_world = None
        self.rover_vel_world = None
        self.rover_yaw_world = None     

        self.drone_pos_world = None
        self.drone_vel_world = None
        self.drone_yaw_world = None
        self.drone_q_enu = None 

        # ---------------- TF Setup ----------------
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- QoS Setup ----------------
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscribers ----------------
        # self.create_subscription(Odometry, '/rover/ekf_odom', self.rover_state_callback, qos_profile)
        # self.create_subscription(Odometry, '/gps/rover_pose_enu', self.rover_state_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.drone_odom_callback, qos_profile)

        # Modified for optitrack testing
        self.create_subscription(PoseStamped, '/tag_priyam/world', self.mocap_pose_callback, qos_profile)
        self.create_subscription(TwistStamped, '/tag_priyam/mocap/twist', self.mocap_twist_callback, qos_profile)

        # ---------------- Publishers ----------------
        self.drone_enu_pub = self.create_publisher(PoseStamped, '/debug/drone_pose_enu', qos_profile)
        self.relative_state_pub = self.create_publisher(Odometry, '/rover/relative_state', qos_profile)

        self.get_logger().info('[ROVER_RELATIVE] Rover Relative State Node has been started.')

        # ---------------- Timer (The "Brain") ----------------
        # Running at 20Hz (0.05s) to ensure low CPU but fresh data for MPC
        self.timer = self.create_timer(0.05, self.compute_relative_state)


    # ----------------- Callbacks ----------------
    # def rover_state_callback(self, msg: Odometry):
    #     '''
    #         Callback for rover state from EKF. 
    #     '''
    #     self.rover_pos_world = np.array([
    #                                         msg.pose.pose.position.x,    
    #                                         msg.pose.pose.position.y,
    #                                         msg.pose.pose.position.z
    #                                     ])
        
    #     q = msg.pose.pose.orientation
    #     _, _, self.rover_yaw_world = euler_from_quaternion([q.x, q.y, q.z, q.w])

    #     self.rover_vel_world = np.array([
    #                                         msg.twist.twist.linear.x,
    #                                         msg.twist.twist.linear.y,
    #                                         msg.twist.twist.linear.z
    #                                     ])     

    # Mocap callbacks for testing without GPS
    def mocap_pose_callback(self, msg: PoseStamped):
        ''' Callback for landing pad pose from OptiTrack '''
        self.rover_pos_world = np.array([
                                            msg.pose.position.x,    
                                            msg.pose.position.y,
                                            msg.pose.position.z
                                        ])
        
        q = msg.pose.orientation
        _, _, self.rover_yaw_world = euler_from_quaternion([q.x, q.y, q.z, q.w])


    def mocap_twist_callback(self, msg: TwistStamped):
        ''' Callback for landing pad velocity from OptiTrack '''
        self.rover_vel_world = np.array([
                                            msg.twist.linear.x,
                                            msg.twist.linear.y,
                                            msg.twist.linear.z
                                        ])

    # -----------------------------------------------------------------------
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
        self.drone_pos_world = np.array([
                                            msg.position[1],   # East
                                            msg.position[0],   # North
                                            -msg.position[2]   # Up
                                        ])

        # Velocity Conversion
        self.drone_vel_world = np.array([
                                            msg.velocity[1],   # East
                                            msg.velocity[0],   # North
                                            -msg.velocity[2]   # Up
                                        ])

        # Yaw Conversion (NED to ENU)
        q_ned = msg.q
        self.drone_q_enu = [q_ned[1], q_ned[0], -q_ned[2], q_ned[3]]

        _, _, yaw_enu = euler_from_quaternion(self.drone_q_enu)
        self.drone_yaw_world = wrap_angle(yaw_enu)


    # ----------------- Main Compute Loop (20Hz) -----------------
    def compute_relative_state(self):
        ''' Performs all math and publishing at a fixed, reliable rate. '''
        
        # Safety Check: Do we have all necessary data?
        if self.drone_pos_world is None or self.rover_pos_world is None:
            return

        now = self.get_clock().now().to_msg()

        # 1. Publish Drone ENU Pose (for Debugging/RVIZ)
        drone_pose_msg = PoseStamped()
        drone_pose_msg.header.stamp = now
        drone_pose_msg.header.frame_id = self.world_frame
        drone_pose_msg.pose.position.x = float(self.drone_pos_world[0])
        drone_pose_msg.pose.position.y = float(self.drone_pos_world[1])
        drone_pose_msg.pose.position.z = float(self.drone_pos_world[2])
        drone_pose_msg.pose.orientation.x = float(self.drone_q_enu[0])
        drone_pose_msg.pose.orientation.y = float(self.drone_q_enu[1])
        drone_pose_msg.pose.orientation.z = float(self.drone_q_enu[2])
        drone_pose_msg.pose.orientation.w = float(self.drone_q_enu[3])
        # self.drone_enu_pub.publish(drone_pose_msg)

        # 2. Broadcast Drone TF
        t_drone = TransformStamped()
        t_drone.header.stamp = now
        t_drone.header.frame_id = self.world_frame
        t_drone.child_frame_id = self.drone_frame
        t_drone.transform.translation.x = drone_pose_msg.pose.position.x
        t_drone.transform.translation.y = drone_pose_msg.pose.position.y
        t_drone.transform.translation.z = drone_pose_msg.pose.position.z
        t_drone.transform.rotation = drone_pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t_drone)

        # 3. RELATIVE MATH (Optimized Local Quaternion)
        # World -> Body rotation = Inverse of Body -> World (drone_q_enu)
        q_world_to_body = [-self.drone_q_enu[0], -self.drone_q_enu[1], -self.drone_q_enu[2], self.drone_q_enu[3]]
        R_world_drone = quaternion_matrix(q_world_to_body)[:3, :3]

        # Relative Pose & Vel
        p_rel_world = self.rover_pos_world - self.drone_pos_world
        p_rel_drone = R_world_drone @ p_rel_world

        # Velocity check (fallback to zero if twist not received yet)
        rv_world = self.rover_vel_world if self.rover_vel_world is not None else np.zeros(3)
        v_rel_world = rv_world - self.drone_vel_world
        v_rel_drone = R_world_drone @ v_rel_world

        # Relative Yaw
        yaw_relative = wrap_angle(self.rover_yaw_world - self.drone_yaw_world)

        # 4. PUBLISH RELATIVE DATA
        rel_state_msg = Odometry()
        rel_state_msg.header.stamp = now
        rel_state_msg.header.frame_id = self.drone_frame
        rel_state_msg.child_frame_id = self.rover_relative_frame

        # Relative Position
        rel_state_msg.pose.pose.position.x = float(p_rel_drone[0])
        rel_state_msg.pose.pose.position.y = float(p_rel_drone[1])
        rel_state_msg.pose.pose.position.z = float(p_rel_drone[2])

        # Relative Yaw (Convert Euler back to Quaternion for the message)
        rq = quaternion_from_euler(0, 0, yaw_relative)
        rel_state_msg.pose.pose.orientation.x = rq[0]
        rel_state_msg.pose.pose.orientation.y = rq[1]
        rel_state_msg.pose.pose.orientation.z = rq[2]
        rel_state_msg.pose.pose.orientation.w = rq[3]

        # Relative Velocity
        rel_state_msg.twist.twist.linear.x = float(v_rel_drone[0])
        rel_state_msg.twist.twist.linear.y = float(v_rel_drone[1])
        rel_state_msg.twist.twist.linear.z = float(v_rel_drone[2])

        # Publish the single combined message
        self.relative_state_pub.publish(rel_state_msg)

        # 4. BROADCAST RELATIVE TF (Drone -> Rover Relative)
        tf_out = TransformStamped()
        tf_out.header.stamp = now
        tf_out.header.frame_id = self.drone_frame
        tf_out.child_frame_id = self.rover_relative_frame
        tf_out.transform.translation.x = rel_state_msg.pose.pose.position.x
        tf_out.transform.translation.y = rel_state_msg.pose.pose.position.y
        tf_out.transform.translation.z = rel_state_msg.pose.pose.position.z
        tf_out.transform.rotation = rel_state_msg.pose.pose.orientation
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