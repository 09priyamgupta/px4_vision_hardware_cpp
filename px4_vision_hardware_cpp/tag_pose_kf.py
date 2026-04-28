#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32MultiArray


class TagPoseKF(Node):
    '''
        Kalman Filter for AprilTag pose estimation (STATIC TAG).

        Assumptions:
        - AprilTag is static in the world frame
        - Measurements are noisy but unbiased
        - Goal is noise reduction (not prediction)

        State:
            x = [x, y, z]^T

        Process model (constant position):
            x_k+1 = x_k + w_k

        Measurement model:
            z_k = x_k + v_k
    '''
    def __init__(self):
        super().__init__('tag_pose_kf_static')

        # ---------------- Parameters ----------------
        self.declare_parameter('control_rate', 20.0)                # Hz
        self.declare_parameter('frames.world', 'map')

        self.kf_rate = self.get_parameter('kf_rate').value
        self.world_frame = self.get_parameter('frames.world').value

        # ---------------- KF State ----------------
        self.x = np.zeros((3,1))    # State vector: [x, y, z]
        self.P = np.eye(3) * 1.0    # Initial covariance

        self.initialized = False
        self.dt = 1.0 / self.kf_rate

        # ---------------- Measurement ----------------
        self.z = None               # Measurement vector: [x, y, z]

        # ---------------- Noise Models ----------------
        # Small process noise => allows slow correction, avoids drift
        self.Q = np.eye(3) * 1e-3                  

        # Measurement noise from AprilTag detection
        self.R = np.diag([0.01, 0.01, 0.01])       

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Publishers ----------------
        self.filtered_tag_pub = self.create_publisher(PoseStamped, '/landing/tag_pose_map_kf_filtered', 10)
        self.tag_cov_pub = self.create_publisher(Float32MultiArray, '/landing/tag_pose_kf_covariance', 10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(PoseStamped, '/landing/tag_pose_map', self.tag_pose_callback, qos)

        # ---------------- Timer ----------------
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self.kf_step)

        self.get_logger().info("[TAG_POSE_KF] Tag Pose Kalman Filter Node has been started.")


    # ---------------- Callbacks ----------------
    def tag_pose_callback(self, msg: PoseStamped):
        '''
            Callback for tag pose measurements.
            - Input of Kalman Filter = Raw AprilTag pose in MAP frame
        '''
        self.z = np.array([[msg.pose.position.x],
                           [msg.pose.position.y],
                           [msg.pose.position.z]])
        
        # Initialize the Kalman Filter with the first measurement
        if not self.initialized:
            self.x = self.z.copy()              # Set position
            self.P = np.eye(3)                  # Reset covariance
            self.initialized = True
            self.get_logger().info('[TAG_POSE_KF] Initialized with first measurement.')

    
    # ---------------- Utilities ----------------
    def publish_filtered_tag_pose(self):
        '''
            Publish the filtered tag pose.
        '''
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        
        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.position.z = float(self.x[2])

        msg.pose.orientation.w = 1.0                # Neutral orientation

        self.filtered_tag_pub.publish(msg)

    def publish_tag_covariance(self):
        '''
            Publish the tag pose covariance.
        '''
        cov_msg = Float32MultiArray()
        cov_msg.data = [
                            float(self.P[0,0]), 
                            float(self.P[1,1]),
                            float(self.P[2,2])                
                        ]

        self.tag_cov_pub.publish(cov_msg)

    
    # ---------------- Kalman Filter Step ----------------
    def kf_step(self):
        '''
            Perform a Kalman Filter step: prediction and update.
        '''
        if not self.initialized or self.z is None:
            return

        # ------------------- Prediction Step -------------------
        # x_k+1 = x_k (constant position)
        F = np.eye(3)                           # F = State transition matrix

        self.x = F @ self.x                     # State prediction
        self.P = F @ self.P @ F.T + self.Q      # Covariance prediction

        # ------------------- Update Step -------------------
        H = np.eye(3)                           # H = Measurement matrix

        y = self.z - H @ self.x                 # Measurement residual / innovation
        S = H @ self.P @ H.T + self.R           # Residual covariance

        K = self.P @ H.T @ np.linalg.inv(S)     # Kalman gain

        self.x = self.x + K @ y                 # State update
        self.P = (np.eye(3) - K @ H) @ self.P   # Covariance update

        trace_P = np.trace(self.P)

        self.get_logger().debug(f"[TAG_POSE_KF] Update | Innovation norm: {np.linalg.norm(y):.3f}")
        self.get_logger().debug(f"[TAG_POSE_KF] Update | Trace of P: {trace_P:.4f}")

        # ------------------- Publish Filtered Pose -------------------
        self.publish_filtered_tag_pose()
        self.publish_tag_covariance()


# ---------------- Main ----------------
def main():
    rclpy.init()
    tag_pose_kf_node = TagPoseKF()

    try:
        rclpy.spin(tag_pose_kf_node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        tag_pose_kf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()