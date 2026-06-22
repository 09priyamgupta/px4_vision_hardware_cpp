#!/usr/bin/env python3

import os
import time
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Import message types
from px4_msgs.msg import VehicleOdometry
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped  # Standard MoCap message type

class FOVCalibrationLogger(Node):
    def __init__(self):
        super().__init__('fov_calibration_logger')

        # ---------------- Create CSV File ----------------
        log_dir = os.path.expanduser('~/px4_ros2_ws/src/px4_vision_hardware_cpp/tag_detection_analysis')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_filename = os.path.join(log_dir, f'fov_calibration_{int(time.time())}.csv')

        self.file = open(self.csv_filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # CSV Headers - Added MoCap Altitude
        self.writer.writerow([
            'timestamp', 'px4_alt_m', 'mocap_alt_m', 'num_tags_visible',
            'tag_19_visible', 'bundle_visible', 'detected_ids'
        ])

        # ---------------- State Variables ----------------
        self.px4_alt = -1.0    # Initialize with invalid altitude
        self.mocap_alt = -1.0  # Initialize with invalid altitude

        # ---------------- Subscribers ----------------
        # 1. Odometry for PX4 Estimated Altitude
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_cb,
            qos_profile_sensor_data
        )

        # 2. MoCap for Ground Truth Altitude
        # NOTE: Change '/mocap/pose' to your actual MoCap topic if different
        self.mocap_sub = self.create_subscription(
            PoseStamped,
            '/x500_2_priyam/world', 
            self.mocap_cb,
            10
        )

        # 3. AprilTag Detections
        self.det_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.det_cb,
            10  # Standard reliable QoS for standard ROS 2 topics
        )

        self.get_logger().info(f"[FOV LOGGER] Active! Manually lower the drone over the pad.")
        self.get_logger().info(f"[FOV LOGGER] Saving data to: {self.csv_filename}")

    def odom_cb(self, msg):
        # NED Frame: Z is Down (Positive). To get Altitude, negate it.
        self.px4_alt = -msg.position[2]

    def mocap_cb(self, msg):
        # Assuming ENU Frame from MoCap: Z is Up (Positive Altitude)
        self.mocap_alt = msg.pose.position.z

    def det_cb(self, msg):
        # Do not log if we haven't received altitude data yet
        if self.px4_alt == -1.0 or self.mocap_alt == -1.0:
            return 

        parsed_ids = []
        for det in msg.detections:
            # Depending on the specific apriltag_msgs version in Jazzy/Humble, 
            # id might be a single int or an array. This handles both safely:
            if hasattr(det, 'id') and isinstance(det.id, int):
                parsed_ids.append(det.id)
            elif hasattr(det, 'id') and len(det.id) > 0:
                parsed_ids.append(det.id[0])

        # ---------------- Boolean Logic ----------------
        # Is Tag 19 visible?
        tag_19_visible = 19 in parsed_ids
        
        # Is the Outer Bundle visible? 
        # (Assume any tag ID detected that IS NOT 19 belongs to the outer bundle)
        bundle_tags = [tid for tid in parsed_ids if tid != 19]
        bundle_visible = len(bundle_tags) > 0

        # Log to CSV at the exact moment a camera frame is processed
        self.writer.writerow([
            time.time(),
            round(self.px4_alt, 3),
            round(self.mocap_alt, 3),  # Log Ground Truth Altitude
            len(parsed_ids),
            int(tag_19_visible),
            int(bundle_visible),
            str(parsed_ids)
        ])

    def destroy_node(self):
        self.get_logger().info(f"Closing CSV log file...")
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FOVCalibrationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()