#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleOdometry

import csv
import os
import math
from datetime import datetime

class TFStateLogger(Node):
    '''
        Unified experiment logger for rover trajectory and MPC validation.
        Logs data required for PlotJuggler visualization and PPT presentations.
    '''
    def __init__(self):
        super().__init__('tf_state_logger')

        # ---------------- Parameters ----------------
        self.declare_parameter('control_rate', 20.0)
        self.log_rate_hz = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.log_rate_hz
        
        # ---------------- State storage ----------------
        # Commanded
        self.cmd_xy = [0.0, 0.0]

        # Drone State
        self.drone_ned = [math.nan, math.nan, math.nan]
        self.drone_vel_ned = [math.nan, math.nan]
        self.drone_enu = [math.nan, math.nan, math.nan]

        # Rover Ground Truth
        self.ekf_enu = [math.nan, math.nan, math.nan]
        self.ekf_vel_enu = [math.nan, math.nan]

        # Vision & Filter State
        self.tag_raw_enu = [math.nan, math.nan]
        self.tag_kf_ned = [math.nan, math.nan, math.nan, math.nan] # [x, y, vx, vy]
        self.tag_ema_vel = [math.nan, math.nan] # [vx, vy]
        self.tag_visible = False

        # MPC & Landing Metrics
        self.tracking_errors = [math.nan, math.nan, math.nan] # [Horiz_Err, Allowed_Err, Target_Alt]

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscriptions ----------------
        # 1. Ground Truth & Commands
        self.create_subscription(Twist, '/r1/cmd_vel', self.cmd_callback, qos)
        self.create_subscription(Odometry, '/rover/ekf_odom', self.ekf_callback, qos)
        
        # 2. Drone Telemetry
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.drone_odom_callback, qos)
        
        # 3. Vision Raw
        self.create_subscription(PoseStamped, '/landing/tag_pose_map', self.tag_map_callback, qos)
        self.create_subscription(Bool, '/landing/tag_visible_flag', self.tag_visible_callback, qos)

        # 4. Controller Internal States (Requires publishing from VisionGuidanceController)
        self.create_subscription(Odometry, '/mpc/debug/kf_state', self.kf_state_callback, qos)
        self.create_subscription(Vector3, '/mpc/debug/ema_vel', self.ema_vel_callback, qos)
        self.create_subscription(Vector3, '/mpc/debug/errors', self.errors_callback, qos)

        # --------------------- CSV Logger Setup ---------------------
        self.declare_parameter('log_dir', '~/px4_logs') 
        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)

        filename = 'mpc_landing_log.csv'
        self.log_filepath = os.path.join(log_dir, filename)

        self.csv_file = open(self.log_filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write Headers
        self.csv_writer.writerow([
            'time',
            # Drone
            'drone_x_ned', 'drone_y_ned', 'drone_z_ned', 'drone_vx_ned', 'drone_vy_ned',
            'drone_x_enu', 'drone_y_enu', 'drone_z_enu',
            # Rover Truth
            'rover_x_enu', 'rover_y_enu', 'rover_vx_enu', 'rover_vy_enu',
            # Tag Vision
            'tag_raw_x_enu', 'tag_raw_y_enu', 'tag_visible',
            # Filter Outputs
            'kf_tag_x_ned', 'kf_tag_y_ned', 'kf_tag_vx_ned', 'kf_tag_vy_ned',
            'ema_tag_vx_ned', 'ema_tag_vy_ned',
            # MPC & Errors
            'horizontal_error_m', 'allowed_fov_error_m', 'target_altitude_m'
        ])

        # ---------------- Timer ----------------
        self.create_timer(self.dt, self.log_state)
        self.get_logger().info(f'[TF STATE LOGGER] Started (logging at {self.log_rate_hz} Hz)')
        self.get_logger().info(f'[TF STATE LOGGER] Writing logs to: {self.log_filepath}')

    # ---------------- Callbacks ----------------
    def cmd_callback(self, msg: Twist):
        self.cmd_xy[0] += msg.linear.x * self.dt
        self.cmd_xy[1] += msg.linear.y * self.dt

    def ekf_callback(self, msg: Odometry):
        self.ekf_enu = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.ekf_vel_enu = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]

    def drone_odom_callback(self, msg: VehicleOdometry):
        # NED
        self.drone_ned = [msg.position[0], msg.position[1], msg.position[2]]
        self.drone_vel_ned = [msg.velocity[0], msg.velocity[1]]
        # Convert NED to ENU for plotting: X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
        self.drone_enu = [msg.position[1], msg.position[0], -msg.position[2]]

    def tag_map_callback(self, msg: PoseStamped):
        self.tag_raw_enu = [msg.pose.position.x, msg.pose.position.y]

    def tag_visible_callback(self, msg: Bool):
        self.tag_visible = msg.data

    def kf_state_callback(self, msg: Odometry):
        # Packing KF state into Odometry msg (Pose = Pos, Twist = Vel)
        self.tag_kf_ned = [msg.pose.pose.position.x, msg.pose.pose.position.y, 
                           msg.twist.twist.linear.x, msg.twist.twist.linear.y]

    def ema_vel_callback(self, msg: Vector3):
        self.tag_ema_vel = [msg.x, msg.y]

    def errors_callback(self, msg: Vector3):
        self.tracking_errors = [msg.x, msg.y, msg.z]

    # ---------------- Logging ----------------
    def log_state(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        row = [
            t,
            *self.drone_ned, *self.drone_vel_ned,
            *self.drone_enu,
            *self.ekf_enu[:2], *self.ekf_vel_enu,
            *self.tag_raw_enu, int(self.tag_visible),
            *self.tag_kf_ned,
            *self.tag_ema_vel,
            *self.tracking_errors
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        self.get_logger().info('[LOGGER] Closing CSV file.')
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = TFStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()