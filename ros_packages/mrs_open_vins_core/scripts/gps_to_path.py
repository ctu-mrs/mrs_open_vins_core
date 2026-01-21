#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovariance
from tf2_ros import TransformBroadcaster
import math

def create_pose_with_covariance(pose_stamped: PoseStamped, nav_sat_fix: NavSatFix) -> PoseWithCovariance:
    """
    Create a PoseWithCovariance message from PoseStamped and NavSatFix.
    
    Args:
        pose_stamped: PoseStamped message with position and orientation
        nav_sat_fix: NavSatFix message with GPS covariance data
        
    Returns:
        PoseWithCovariance message with GPS covariance
    """
    pose_with_cov = PoseWithCovariance()
    
    # Copy the pose
    pose_with_cov.pose = pose_stamped.pose
    
    # Initialize covariance matrix with zeros
    covariance = [0.0] * 36
    
    # Map GPS position covariance (3x3) to the pose covariance (6x6)
    # The GPS covariance is for [x, y, z] in the first 3x3 block
    
    # Row 0 (x position)
    covariance[0] = nav_sat_fix.position_covariance[0]   # x variance
    covariance[1] = nav_sat_fix.position_covariance[1]   # x-y covariance
    covariance[2] = nav_sat_fix.position_covariance[2]   # x-z covariance
    
    # Row 1 (y position)
    covariance[6] = nav_sat_fix.position_covariance[3]   # y-x covariance
    covariance[7] = nav_sat_fix.position_covariance[4]   # y variance
    covariance[8] = nav_sat_fix.position_covariance[5]   # y-z covariance
    
    # Row 2 (z position)
    covariance[12] = nav_sat_fix.position_covariance[6]  # z-x covariance
    covariance[13] = nav_sat_fix.position_covariance[7]  # z-y covariance
    covariance[14] = nav_sat_fix.position_covariance[8]  # z variance
    
    # Orientation covariance (rows 3-5): GPS doesn't provide orientation uncertainty
    # Set to large values to indicate unknown/high uncertainty
    covariance[21] = 999999.0  # roll variance (unknown)
    covariance[28] = 999999.0  # pitch variance (unknown)
    covariance[35] = 999999.0  # yaw variance (unknown)
    
    pose_with_cov.covariance = covariance
    
    return pose_with_cov

class GPSToPathConverter(Node):
    def __init__(self):
        super().__init__('gps_to_path_converter')
        
        # Parameters
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('path_topic', '/gps/path')
        self.declare_parameter('pose_topic', '/gps/pose')
        self.declare_parameter('pose_with_cov_topic', '/gps/pose_with_cov')
        self.declare_parameter('path_frame', 'uav1/global')
        self.declare_parameter('gps_frame', 'gps')
        self.declare_parameter('max_path_length', 1000)  # Maximum number of poses to keep
        self.declare_parameter('origin_lat', None)  # If None, uses first GPS point as origin
        self.declare_parameter('origin_lon', None)
        self.declare_parameter('origin_alt', None)
        self.declare_parameter('publish_tf', True)  # Whether to publish TF
        
        # Get parameters
        gps_topic = self.get_parameter('gps_topic').value
        path_topic = self.get_parameter('path_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        pose_with_cov_topic = self.get_parameter('pose_with_cov_topic').value
        self.path_frame = self.get_parameter('path_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.origin_alt = self.get_parameter('origin_alt').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Initialize path
        self.path = Path()
        self.path.header.frame_id = self.path_frame
        
        # Initialize TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store current position for TF
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Origin flag
        self.origin_set = False
        if self.origin_lat is not None and self.origin_lon is not None:
            self.origin_set = True
            if self.origin_alt is None:
                self.origin_alt = 0.0
            self.get_logger().info(f'Using predefined origin: ({self.origin_lat}, {self.origin_lon}, {self.origin_alt})')
        
        # Create subscriber and publisher
        self.gps_sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            10
        )
        
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.pose_with_cov_pub = self.create_publisher(PoseWithCovariance, pose_with_cov_topic, 10)
        
        self.get_logger().info(f'GPS to Path Converter started')
        self.get_logger().info(f'Subscribing to: {gps_topic}')
        self.get_logger().info(f'Publishing to: {path_topic}')
        self.get_logger().info(f'Path frame: {self.path_frame}')
        self.get_logger().info(f'GPS frame: {self.gps_frame}')
        self.get_logger().info(f'Publishing TF: {self.publish_tf}')
        self.get_logger().info(f'Max path length: {self.max_path_length}')
    
    def gps_to_local(self, lat, lon, alt):
        """
        Convert GPS coordinates to local ENU (East-North-Up) coordinates.
        Uses simple equirectangular projection (good for small areas).
        """
        if not self.origin_set:
            # Use first GPS point as origin
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.origin_set = True
            self.get_logger().info(f'Origin set to first GPS point: ({lat:.6f}, {lon:.6f}, {alt:.2f})')
            return 0.0, 0.0, 0.0
        
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        
        # Calculate differences
        dlat = lat_rad - origin_lat_rad
        dlon = lon_rad - origin_lon_rad
        
        # Simple equirectangular projection
        # x = East, y = North, z = Up
        x = R * dlon * math.cos(origin_lat_rad)  # East
        y = R * dlat                              # North
        z = alt - self.origin_alt                 # Up
        
        return x, y, z
    
    def broadcast_transform(self, stamp):
        """Broadcast transform from map to gps frame"""
        if not self.publish_tf:
            return
        
        t = TransformStamped()
        
        # Header
        t.header.stamp = stamp
        t.header.frame_id = self.path_frame
        t.child_frame_id = self.gps_frame
        
        # Translation (current GPS position in local coordinates)
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = self.current_z
        
        # Rotation (identity quaternion - no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    
    def gps_callback(self, msg):
        """Callback for GPS messages"""
        
        # Check if GPS has valid fix
        if msg.status.status < 0:
            self.get_logger().warn('GPS has no fix, skipping...', throttle_duration_sec=5.0)
            return
        
        # Convert GPS to local coordinates
        x, y, z = self.gps_to_local(msg.latitude, msg.longitude, msg.altitude)
        
        # Update current position
        self.current_x = x
        self.current_y = y
        self.current_z = z
        
        # Broadcast TF
        self.broadcast_transform(msg.header.stamp)
        
        # Create PoseStamped
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.path_frame
        
        # Position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Orientation (identity quaternion - no rotation)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        # Add to path
        self.path.poses.append(pose)
        
        # Limit path length
        # if len(self.path.poses) > self.max_path_length:
        #     self.path.poses.pop(0)
        
        # Update path header timestamp
        self.path.header.stamp = msg.header.stamp
        
        # Publish path
        self.path_pub.publish(self.path)
        self.pose_pub.publish(pose)

        gps_pose_with_cov = create_pose_with_covariance(pose, msg)
        self.pose_with_cov_pub.publish(gps_pose_with_cov)
        
        self.get_logger().debug(
            f'Added pose to path: ({x:.2f}, {y:.2f}, {z:.2f}) - Total poses: {len(self.path.poses)}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GPSToPathConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
