#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from pyproj import Proj


class GpsRoverEnuPublisher(Node):
    '''
        This node subscribes to the GPS position of the rover and the home position, 
        converts the GPS coordinates to ENU frame, and publishes the rover's position
        in the ENU frame. 
        It also broadcasts a TF from 'map' to 'rover_base_link'.
    '''
    def __init__(self):
        super().__init__('gps_rover_enu_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.rover_odom', 'r1_rover/odom')

        self.world_frame = self.get_parameter('frames.world').value
        self.rover_odom_frame = self.get_parameter('frames.rover_odom').value

        self.get_logger().info(f"[FRAMES] world={self.world_frame}, rover_odom={self.rover_odom_frame}")

        # ---------------- State storage ----------------
        self.rover_gps = None               # Latest GPS reading of the rover
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None

        self.proj = None
        self.proj_initialized = False

        # ---------------- Calibration offsets (if needed) ----------------
        self.initial_offset_z = None

        # ---------------- QOS Setup ----------------
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(NavSatFix, '/r1/gps/fix', self.rover_gps_callback, qos_profile)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        # ---------------- Publishers ----------------
        self.rover_enu_pub = self.create_publisher(PoseStamped, '/gps/rover_pose_enu', qos_profile)

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.1, self.update)           # 10 Hz

        self.get_logger().info('[GPS_ENU] GPS Rover ENU Publisher Node has been started.')


    # ---------------- Callbacks ----------------
    def rover_gps_callback(self, msg: NavSatFix):
        '''
            Callback for rover GPS readings.
        '''
        self.rover_gps = msg

    def local_pos_callback(self, msg: VehicleLocalPosition):
        '''
            Callback for Drone's Local Position.
            We need the ref_lat and ref_lon to match the drone's (0,0) origin.
        '''
        # Check if the global reference is valid (xy_global is a boolean flag in PX4)
        if msg.xy_global:
            self.ref_lat = msg.ref_lat
            self.ref_lon = msg.ref_lon
            self.ref_alt = msg.ref_alt

            if not self.proj_initialized:
                self.init_projection()

    
    # ---------------- Helper Functions ----------------
    def init_projection(self):
        '''
            Initialize the pyproj projection for ENU conversion using the home position as the origin.
        '''
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().warn('[GPS_ENU] Cannot initialize projection: No global reference from Drone yet.', throttle_duration_sec=1.0)
            return

        self.proj = Proj(proj='aeqd', lat_0=self.ref_lat, lon_0=self.ref_lon, datum='WGS84', units='m')
        self.proj_initialized = True
        self.get_logger().info(f'[GPS_ENU] Local ENU projection initialized at lat={self.ref_lat:.7f}, lon={self.ref_lon:.7f}.')

    # ---------------- Main Update Loop ----------------
    def update(self):
        '''
            Main update loop that runs at 10 Hz. It converts the latest GPS reading to ENU and publishes it, 
            and also broadcasts the TF from 'map' to 'rover_base_link'.
        '''
        if not self.proj_initialized or self.rover_gps is None:
            return
        
        # Convert Rover GPS to ENU (Relative to Drone's Origin)
        x_east, y_north = self.proj(self.rover_gps.longitude, self.rover_gps.latitude)
        
        # Altitude relative to Drone's Reference Altitude
        raw_z = self.rover_gps.altitude - self.ref_alt

        # Perform Calibration on first run
        if self.initial_offset_z is None:
            self.initial_offset_z = raw_z
            self.get_logger().info(f'[GPS_ENU] Auto-calibrated Z offset: {self.initial_offset_z:.3f} m. Rover is now at Z=0.')

        # Apply calibration
        z_up = raw_z - self.initial_offset_z

        now = self.get_clock().now().to_msg()

        # Publish ENU position
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.world_frame
        
        pose_msg.pose.position.x = x_east
        pose_msg.pose.position.y = y_north
        pose_msg.pose.position.z = z_up

        pose_msg.pose.orientation.w = 1.0           # No yaw info from GPS

        self.rover_enu_pub.publish(pose_msg)


# ------------------- Main Function ----------------
def main():
    rclpy.init()
    node = GpsRoverEnuPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
