#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# Allow callbacks to be executed in parallel without restriction
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from px4_vision_hardware_cpp.utils.ekf_core import EKF
from px4_vision_hardware_cpp.utils.motion_models import rover_motion_model, rover_jacobian_F
from px4_vision_hardware_cpp.utils.common import wrap_angle


class RoverStateEKFNode(Node):
    '''
        Multi-source fusion EKF node for precision landing.
        This node fuses GPS position, odometry velocity, and IMU yaw rate measurements to estimate 
        the rover's state (position, velocity, yaw) in the world frame and AprilTag for precision landing.
    '''
    def __init__(self):
        super().__init__('rover_state_ekf_node')

        # Allow callbacks to be executed in parallel without restriction
        cb = ReentrantCallbackGroup()    

        # ----------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.rover_base', 'r1_rover/base_link')

        self.world_frame = self.get_parameter('frames.world').value
        self.rover_base_frame = self.get_parameter('frames.rover_base').value

        # ---------------- EKF Initialization ----------------
        x0 = np.zeros(5)                            # Initial state: [x, y, vx, vy, yaw]
        P0 = np.diag([10.0, 10.0, 1.0, 1.0, 0.5])   # Initial covariance

        # Process noise covariance - Rover motion uncertainty
        Q = np.diag([0.05, 0.05, 0.2, 0.2, 0.05])   

        self.ekf = EKF(x0, P0, Q, None)  
        self.ekf_initialized = False

        # Flag to track if we have aligned the yaw to the motion direction yet
        self.yaw_aligned_to_motion = False

        # ----------------- Tuning Matrices (The "Trust" levels for each sensor) ----------------
        self.R_gps = np.diag([1.5, 1.5])            # Measurement noise covariance for GPS
        
        self.R_vision_pos = np.diag([0.05, 0.05])   # Measurement noise covariance for vision (AprilTag)
        self.R_vision_yaw = np.array([[0.05]])

        self.R_odom = np.diag([0.1, 0.1])           # Measurement noise covariance for odometry
        self.R_motion_yaw = np.array([[0.1]])       # Trust level for "Course Over Ground"

        # ---------------- Sensor Buffers ----------------
        self.gps_xy = None
        self.last_gps_xy = None
        self.odom_vel = None
        self.imu_yaw_rate = 0.0

        self.vision_pose_msg = None
        self.vision_new = False

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscribers ----------------
        self.create_subscription(PoseStamped, '/gps/rover_pose_enu', self.gps_callback, qos, callback_group=cb)
        self.create_subscription(PoseStamped, '/apriltag/relative_pose', self.vision_callback, qos, callback_group=cb)
        
        self.create_subscription(Odometry, '/r1/odom', self.odom_callback, qos, callback_group=cb)
        self.create_subscription(Imu, '/r1/imu', self.imu_callback, qos, callback_group=cb)

        # ---------------- Publishers ----------------
        self.state_pub = self.create_publisher(Odometry, '/rover/ekf_odom', qos)

        # ---------------- Timer for EKF Prediction and Update ----------------
        self.declare_parameter('control_rate', 20.0)
        control_rate = self.get_parameter('control_rate').value
        dt = 1.0 / control_rate         # 20 Hz

        self.timer = self.create_timer(dt, self.ekf_update, callback_group=cb)           # 20 Hz
        self.last_time = self.get_clock().now()
        self.create_timer(dt, self.ekf_update, callback_group=cb)           # 20 Hz

        self.get_logger().info('[ROVER_EKF] Fusion Engine Started.')


    # ----------------- Callbacks ----------------
    def gps_callback(self, msg: PoseStamped):
        '''
            GPS callback to update position measurements.
        '''
        current_xy = np.array([msg.pose.position.x, msg.pose.position.y])
        self.gps_xy = current_xy

        # Initialize EKF if this is the first GPS measurement we get
        if not self.ekf_initialized:
            self.ekf.x[0] = self.gps_xy[0]
            self.ekf.x[1] = self.gps_xy[1]
            self.ekf_initialized = True
            self.last_gps_xy = current_xy
            self.get_logger().info(f'[ROVER_EKF] Initialized state from GPS: x={self.ekf.x[0]:.2f}, y={self.ekf.x[1]:.2f}')

        if self.last_gps_xy is not None:
            dx = current_xy[0] - self.last_gps_xy[0]
            dy = current_xy[1] - self.last_gps_xy[1]
            dist = np.sqrt(dx*dx + dy*dy)

            # Only calculate heading if we moved significantly (> 15cm)
            if dist > 0.15:
                motion_yaw = np.arctan2(dy, dx)
                
                # Check for "Reverse" driving (Optional robustness)
                # If motion_yaw is ~180 deg opposite to current estimate, we might be reversing.
                # For now, assuming forward driving for simplicity.

                # 1. Snap Initialization: If we haven't aligned yet, force the EKF yaw
                if not self.yaw_aligned_to_motion:
                    self.ekf.x[4] = motion_yaw
                    self.yaw_aligned_to_motion = True
                    self.get_logger().warn(f'[ROVER_EKF] Yaw Initialized from Motion: {np.degrees(motion_yaw):.1f} deg')
                
                # 2. Continuous Fusion: Update EKF with this "Motion Yaw"
                else:
                    # We create a pseudo-measurement for yaw
                    def h_yaw(x): return np.array([x[4]])
                    H_yaw = np.array([[0, 0, 0, 0, 1]])
                    z_yaw = np.array([motion_yaw])
                    
                    # Fuse it!
                    self.ekf.update(z_yaw, h_yaw, H_yaw, self.R_motion_yaw)

                # Update the last position anchor
                self.last_gps_xy = current_xy

    def vision_callback(self, msg: PoseStamped):
        ''' 
            Corrects EKF Position and Yaw when AprilTag is seen.
            Transforms Tag Pose (relative) -> Map Pose (Global).
        '''
        if not self.ekf_initialized: return

        try:
            # 1. Get Transform from Map -> Camera (Frame of the message)
            # The message header frame is usually the camera optical frame
            trans = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, rclpy.time.Time())
            
            # 2. Transform the Pose to Map Frame
            # This gives us the absolute position/orientation of the Tag in the Map
            global_tag_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
            
            # 3. Extract Position
            z_pos = np.array([global_tag_pose.position.x, global_tag_pose.position.y])
            
            # 4. Extract Yaw
            q = global_tag_pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            tag_yaw = np.arctan2(siny_cosp, cosy_cosp)

            # 5. Apply Tag-to-Rover Offset correction
            # Based on logs: Tag is ~90 deg (+pi/2) rotated relative to Rover Body
            # So Rover Yaw = Tag Yaw - 90 deg
            rover_yaw_from_vision = wrap_angle(tag_yaw - np.pi/2)

            # --- Update EKF ---
            
            # Update Position (High Trust)
            def h_pos(x): return x[0:2]
            H_pos = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0]])
            self.ekf.update(z_pos, h_pos, H_pos, self.R_vision_pos)

            # Update Yaw (High Trust)
            def h_yaw(x): return np.array([x[4]])
            H_yaw = np.array([[0, 0, 0, 0, 1]])
            self.ekf.update(np.array([rover_yaw_from_vision]), h_yaw, H_yaw, self.R_vision_yaw)
            
            self.vision_new = True

        except Exception as e:
            self.get_logger().warn(f'Vision Transform Error: {e}', throttle_duration_sec=1.0)

    def odom_callback(self, msg: Odometry):
        '''
            Odometry callback to update velocity measurements.
        '''
        # Get the velocity in Body Frame (Forward, Left)
        v_body_x = msg.twist.twist.linear.x
        v_body_y = msg.twist.twist.linear.y

        current_ekf_yaw = self.ekf.x[4]

        # Rotate Velocity to World Frame
        #    vx_world = v_forward * cos(yaw) - v_left * sin(yaw)
        #    vy_world = v_forward * sin(yaw) + v_left * cos(yaw)
        # v_world_x = v_body_x * np.cos(yaw_odom) - v_body_y * np.sin(yaw_odom)
        # v_world_y = v_body_x * np.sin(yaw_odom) + v_body_y * np.cos(yaw_odom)

        v_world_x = v_body_x * np.cos(current_ekf_yaw) - v_body_y * np.sin(current_ekf_yaw)
        v_world_y = v_body_x * np.sin(current_ekf_yaw) + v_body_y * np.cos(current_ekf_yaw)

        # Update the buffer
        self.odom_vel = np.array([v_world_x, v_world_y])

    def imu_callback(self, msg: Imu):
        '''
            IMU callback to update yaw rate measurements.
        '''
        self.imu_yaw_rate = msg.angular_velocity.z

    
    # ----------------- EKF Prediction and Update ----------------
    def ekf_update(self):
        ''''
            Main EKF update loop. 
            1. Predict step using the motion model and IMU yaw rate.
            2. Update step using GPS position and odometry velocity measurements.
        '''
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return
        
        # -------------------- EKF Predict Step ----------------
        u = np.array([self.imu_yaw_rate])       # Control input: [yaw_rate]
        F = rover_jacobian_F(self.ekf.x, dt)

        self.ekf.predict(rover_motion_model, F, u, dt)
        self.ekf.x[4] = wrap_angle(self.ekf.x[4])           # Wrap yaw angle

        # Measurement Matrix: We measure position (x, y) 
        def h_pos(x):
            return x[0:2]           
        
        # Jacobian of the measurement function for GPS - (x, y, vx, vy, yaw)
        H_pos = np.array([
                        [1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0]
                    ])

        # -------------------- Update: GPS (Always available, High Noise) ----------------
        if self.gps_xy is not None:
            z = self.gps_xy             # Measurement: [x, y] from GPS
            self.ekf.update(z, h_pos, H_pos, self.R_gps)

        # -------------------- Update: Odometry Velocity (Always available, Moderate Noise) ----------------
        if self.odom_vel is not None:
            def h_vel(x):
                return x[2:4]           # Measurement function for odometry: [vx, vy]
            
            z = self.odom_vel           # Measurement: [vx, vy] from odometry
            
            # Jacobian of the measurement function for odometry - (x, y, vx, vy, yaw)
            H_vel = np.array([
                            [0, 0, 1, 0, 0],
                            [0, 0, 0, 1, 0]
                        ])
            
            self.ekf.update(z, h_vel, H_vel, self.R_odom)

        # ---------------- Publish Estimated State by EKF ----------------
        self.publish_state(now)

    # ----------------- Publish State ----------------
    def publish_state(self, timestamp):
        '''
            Publish the estimated state as an Odometry message.
        '''
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.world_frame
        odom_msg.child_frame_id = self.rover_base_frame

        odom_msg.pose.pose.position.x = self.ekf.x[0]
        odom_msg.pose.pose.position.y = self.ekf.x[1]

        yaw = self.ekf.x[4]
        odom_msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(yaw / 2.0)

        odom_msg.twist.twist.linear.x = self.ekf.x[2]
        odom_msg.twist.twist.linear.y = self.ekf.x[3]

        # Populate the covariance (6x6 matrix, row-major)
        # We only track x, y, vx, vy, yaw. 
        # Indices: x=0, y=7, z=14, roll=21, pitch=28, yaw=35
        # P is 5x5: [x, y, vx, vy, yaw]
        cov = np.zeros(36)

        # Fill in Position Covariance (x, y)
        cov[0] = self.ekf.P[0, 0]    # Variance in X
        cov[7] = self.ekf.P[1, 1]    # Variance in Y

        # Fill in Yaw Covariance
        cov[35] = self.ekf.P[4, 4]   # Variance in Yaw

        # We can also fill in some covariance for velocity if we want, but it's optional 
        # since many algorithms only use the position and yaw covariance for data association and fusion.

        odom_msg.pose.covariance = cov.tolist()

        self.state_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.rover_base_frame

        t.transform.translation.x = self.ekf.x[0]
        t.transform.translation.y = self.ekf.x[1]
        t.transform.translation.z = 0.0

        yaw = self.ekf.x[4]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(yaw / 2.0)
        t.transform.rotation.w = np.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)


# ----------------- Main Function ----------------
def main():
    rclpy.init()
    node = RoverStateEKFNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()