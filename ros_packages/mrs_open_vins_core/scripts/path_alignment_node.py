#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R


class PathAlignmentNode(Node):
    def __init__(self):
        super().__init__('path_alignment_node')
        
        # Parameters
        self.declare_parameter('min_gps_points', 10)
        self.declare_parameter('min_vio_points', 20)
        self.declare_parameter('gps_start_fraction', 0.1)  # Skip first N% of GPS path
        self.declare_parameter('gps_end_fraction', 0.2)    # Skip last N% of GPS path
        self.declare_parameter('rotation_method', 'least_squares')  # 'least_squares' or 'circular_mean'
        
        self.min_gps_points = self.get_parameter('min_gps_points').value
        self.min_vio_points = self.get_parameter('min_vio_points').value
        self.gps_start_fraction = self.get_parameter('gps_start_fraction').value
        self.gps_end_fraction = self.get_parameter('gps_end_fraction').value
        self.rotation_method = self.get_parameter('rotation_method').value
        
        # Clamp fractions to valid range
        self.gps_start_fraction = max(0.0, min(0.5, self.gps_start_fraction))
        self.gps_end_fraction = max(0.0, min(0.5, self.gps_end_fraction))
        
        # Validate rotation method
        if self.rotation_method not in ['least_squares', 'circular_mean']:
            self.get_logger().warn(f'Invalid rotation_method "{self.rotation_method}", using least_squares')
            self.rotation_method = 'least_squares'
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            Path,
            '/gps/path',
            self.gps_callback,
            10
        )
        
        self.vio_sub = self.create_subscription(
            Path,
            '/uav1/open_vins/pathimu',
            self.vio_callback,
            10
        )
        
        # Publisher
        self.aligned_path_pub = self.create_publisher(
            Path,
            '/uav1/aligned_path',
            10
        )
        
        # Storage
        self.latest_gps_path = None
        self.latest_vio_path = None
        
        self.get_logger().info('Path Alignment Node initialized')
        self.get_logger().info(f'Parameters: min_gps={self.min_gps_points}, '
                              f'min_vio={self.min_vio_points}')
        self.get_logger().info(f'GPS truncation: skip first {self.gps_start_fraction:.1%}, '
                              f'skip last {self.gps_end_fraction:.1%}')
        self.get_logger().info(f'Rotation method: {self.rotation_method}')
    
    def gps_callback(self, msg):
        """Store GPS path and trigger alignment"""
        self.latest_gps_path = msg
        self.compute_and_publish_alignment()
    
    def vio_callback(self, msg):
        """Store VIO path"""
        self.latest_vio_path = msg
        # Note: we don't trigger alignment here, only when GPS arrives
    
    def get_timestamp(self, pose_stamped):
        """Extract timestamp as float seconds from PoseStamped"""
        return pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
    
    def find_closest_vio_pose(self, gps_timestamp, vio_path):
        """
        Find the VIO pose with timestamp closest to the given GPS timestamp.
        
        Args:
            gps_timestamp: Target timestamp (float seconds)
            vio_path: Full VIO path
            
        Returns:
            PoseStamped from VIO path, or None if VIO path is empty
        """
        if vio_path is None or len(vio_path.poses) == 0:
            return None
        
        # Find closest timestamp
        min_diff = float('inf')
        closest_pose = None
        
        for vio_pose in vio_path.poses:
            vio_timestamp = self.get_timestamp(vio_pose)
            diff = abs(vio_timestamp - gps_timestamp)
            
            if diff < min_diff:
                min_diff = diff
                closest_pose = vio_pose
        
        return closest_pose
    
    def truncate_gps_path(self, gps_path):
        """
        Remove first and last portions of GPS path.
        
        Args:
            gps_path: Original GPS path
            
        Returns:
            Truncated GPS path (middle portion only)
        """
        num_poses = len(gps_path.poses)
        start_idx = int(num_poses * self.gps_start_fraction)
        end_idx = num_poses - int(num_poses * self.gps_end_fraction)
        
        # Ensure we have at least 2 poses
        if end_idx - start_idx < 2:
            start_idx = 0
            end_idx = num_poses
        
        truncated = Path()
        truncated.header = gps_path.header
        truncated.poses = gps_path.poses[start_idx:end_idx]
        
        return truncated
    
    def match_vio_to_gps(self, gps_path, vio_path):
        """
        For each GPS pose, find the corresponding VIO pose by timestamp.
        
        Args:
            gps_path: GPS path (already truncated)
            vio_path: Full VIO path
            
        Returns:
            List of matched VIO poses (same length as gps_path)
        """
        matched_vio_poses = []
        
        for gps_pose in gps_path.poses:
            gps_timestamp = self.get_timestamp(gps_pose)
            vio_pose = self.find_closest_vio_pose(gps_timestamp, vio_path)
            
            if vio_pose is not None:
                matched_vio_poses.append(vio_pose)
        
        return matched_vio_poses
    
    def extract_relative_vectors(self, poses):
        """
        Extract relative vectors and distances from a list of poses.
        
        Args:
            poses: List of PoseStamped messages
            
        Returns:
            Tuple of (vectors, distances)
        """
        if len(poses) < 2:
            return None, None
        
        vectors = []
        distances = []
        
        for i in range(1, len(poses)):
            prev_pos = np.array([
                poses[i-1].pose.position.x,
                poses[i-1].pose.position.y,
                poses[i-1].pose.position.z
            ])
            
            curr_pos = np.array([
                poses[i].pose.position.x,
                poses[i].pose.position.y,
                poses[i].pose.position.z
            ])
            
            relative_vec = curr_pos - prev_pos
            distance = np.linalg.norm(relative_vec)
            
            vectors.append(relative_vec)
            distances.append(distance)
        
        return vectors, np.array(distances)
    
    def compute_z_rotation_least_squares(self, gps_vectors, gps_distances,
                                         vio_vectors, vio_distances):
        """
        Compute Z-axis rotation using weighted least squares.
        This minimizes: Σ w[i] × ||R(θ) × vio[i] - gps[i]||²
        
        For 2D rotation, the optimal angle has a closed-form solution:
        θ = atan2(Σ w[i]×b[i], Σ w[i]×a[i])
        where:
          a[i] = vio_x × gps_x + vio_y × gps_y  (dot product in XY)
          b[i] = vio_x × gps_y - vio_y × gps_x  (cross product in XY)
        """
        if len(gps_vectors) == 0 or len(vio_vectors) == 0:
            return None
        
        if len(gps_vectors) != len(vio_vectors):
            self.get_logger().error(f'Vector count mismatch: GPS={len(gps_vectors)}, VIO={len(vio_vectors)}')
            return None
        
        sum_weighted_a = 0.0
        sum_weighted_b = 0.0
        total_weight = 0.0
        valid_pairs = 0
        
        for i in range(len(gps_vectors)):
            # Extract XY components
            vio_x, vio_y = vio_vectors[i][0], vio_vectors[i][1]
            gps_x, gps_y = gps_vectors[i][0], gps_vectors[i][1]
            
            # Skip if either vector is too small
            vio_mag = np.sqrt(vio_x**2 + vio_y**2)
            gps_mag = np.sqrt(gps_x**2 + gps_y**2)
            
            if vio_mag < 1e-6 or gps_mag < 1e-6:
                continue
            
            # Weight by product of distances
            weight = gps_distances[i] * vio_distances[i]
            
            # Compute dot and cross products
            a = vio_x * gps_x + vio_y * gps_y  # dot product
            b = vio_x * gps_y - vio_y * gps_x  # cross product (z-component)
            
            sum_weighted_a += weight * a
            sum_weighted_b += weight * b
            total_weight += weight
            valid_pairs += 1
        
        if valid_pairs == 0 or total_weight < 1e-9:
            return None
        
        # Optimal rotation angle
        rotation = np.arctan2(sum_weighted_b, sum_weighted_a)
        
        self.get_logger().info(f'Least Squares: Used {valid_pairs} vector pairs')
        self.get_logger().info(f'Rotation: {np.degrees(rotation):.2f}°')
        
        return rotation
    
    def compute_z_rotation_circular_mean(self, gps_vectors, gps_distances, 
                                        vio_vectors, vio_distances):
        """
        Compute Z-axis rotation using weighted circular mean.
        This computes average headings and finds the rotation between them.
        Weight = gps_distance × vio_distance
        """
        if len(gps_vectors) == 0 or len(vio_vectors) == 0:
            return None
        
        if len(gps_vectors) != len(vio_vectors):
            self.get_logger().error(f'Vector count mismatch: GPS={len(gps_vectors)}, VIO={len(vio_vectors)}')
            return None
        
        gps_angles = []
        vio_angles = []
        weights = []
        
        for i in range(len(gps_vectors)):
            # GPS heading in XY plane
            gps_xy = gps_vectors[i][:2]
            gps_distance_xy = np.linalg.norm(gps_xy)
            
            # VIO heading in XY plane
            vio_xy = vio_vectors[i][:2]
            vio_distance_xy = np.linalg.norm(vio_xy)
            
            # Skip if either vector is too small
            if gps_distance_xy > 1e-6 and vio_distance_xy > 1e-6:
                gps_angle = np.arctan2(gps_xy[1], gps_xy[0])
                vio_angle = np.arctan2(vio_xy[1], vio_xy[0])
                
                # Weight by product of distances
                weight = gps_distances[i] * vio_distances[i]
                
                gps_angles.append(gps_angle)
                vio_angles.append(vio_angle)
                weights.append(weight)
        
        if len(gps_angles) == 0:
            return None
        
        # Convert to numpy and normalize weights
        gps_angles = np.array(gps_angles)
        vio_angles = np.array(vio_angles)
        weights = np.array(weights)
        weights = weights / np.sum(weights)
        
        # Weighted circular mean
        gps_mean = np.arctan2(
            np.sum(weights * np.sin(gps_angles)),
            np.sum(weights * np.cos(gps_angles))
        )
        vio_mean = np.arctan2(
            np.sum(weights * np.sin(vio_angles)),
            np.sum(weights * np.cos(vio_angles))
        )
        
        # Compute rotation
        rotation = gps_mean - vio_mean
        rotation = np.arctan2(np.sin(rotation), np.cos(rotation))  # Normalize to [-pi, pi]
        
        self.get_logger().info(f'Circular Mean: Used {len(gps_angles)} vector pairs')
        self.get_logger().info(f'GPS heading: {np.degrees(gps_mean):.2f}°, '
                              f'VIO heading: {np.degrees(vio_mean):.2f}°')
        self.get_logger().info(f'Rotation: {np.degrees(rotation):.2f}°')
        
        return rotation
    
    def compute_z_rotation_from_vectors(self, gps_vectors, gps_distances, 
                                        vio_vectors, vio_distances):
        """
        Compute Z-axis rotation using the selected method.
        Dispatches to either least_squares or circular_mean method.
        """
        if self.rotation_method == 'least_squares':
            return self.compute_z_rotation_least_squares(
                gps_vectors, gps_distances, vio_vectors, vio_distances
            )
        elif self.rotation_method == 'circular_mean':
            return self.compute_z_rotation_circular_mean(
                gps_vectors, gps_distances, vio_vectors, vio_distances
            )
        else:
            self.get_logger().error(f'Unknown rotation method: {self.rotation_method}')
            return None
    
    def apply_z_rotation_to_path(self, path, rotation_angle):
        """Apply Z-axis rotation to entire path"""
        if path is None or rotation_angle is None:
            return None
        
        aligned_path = Path()
        aligned_path.header = path.header
        
        rot = R.from_euler('z', rotation_angle)
        
        for pose_stamped in path.poses:
            new_pose = PoseStamped()
            new_pose.header = pose_stamped.header
            
            # Rotate position
            pos = np.array([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            ])
            rotated_pos = rot.apply(pos)
            
            new_pose.pose.position.x = rotated_pos[0]
            new_pose.pose.position.y = rotated_pos[1]
            new_pose.pose.position.z = rotated_pos[2]
            
            # Rotate orientation
            quat = np.array([
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            ])
            original_rot = R.from_quat(quat)
            combined = rot * original_rot
            new_quat = combined.as_quat()
            
            new_pose.pose.orientation.x = new_quat[0]
            new_pose.pose.orientation.y = new_quat[1]
            new_pose.pose.orientation.z = new_quat[2]
            new_pose.pose.orientation.w = new_quat[3]
            
            aligned_path.poses.append(new_pose)
        
        return aligned_path
    
    def compute_and_publish_alignment(self):
        """Main alignment logic"""
        if self.latest_gps_path is None or self.latest_vio_path is None:
            return
        
        # Check minimum points
        if len(self.latest_gps_path.poses) < self.min_gps_points:
            return
        
        if len(self.latest_vio_path.poses) < self.min_vio_points:
            return
        
        # Step 1: Truncate GPS path (remove start and end)
        truncated_gps = self.truncate_gps_path(self.latest_gps_path)
        
        if len(truncated_gps.poses) < 2:
            self.get_logger().warn('Not enough poses after truncation')
            return
        
        self.get_logger().info(f'Truncated GPS: {len(truncated_gps.poses)} poses '
                              f'(from {len(self.latest_gps_path.poses)})')
        
        # Step 2: Match VIO poses to GPS poses by timestamp
        matched_vio_poses = self.match_vio_to_gps(truncated_gps, self.latest_vio_path)
        
        if len(matched_vio_poses) < 2:
            self.get_logger().warn('Could not match enough VIO poses to GPS')
            return
        
        self.get_logger().info(f'Matched {len(matched_vio_poses)} VIO poses to GPS')
        
        # Step 3: Extract relative vectors from both matched paths
        gps_vectors, gps_distances = self.extract_relative_vectors(truncated_gps.poses)
        vio_vectors, vio_distances = self.extract_relative_vectors(matched_vio_poses)
        
        if gps_vectors is None or vio_vectors is None:
            self.get_logger().warn('Could not extract vectors')
            return
        
        # Step 4: Compute rotation from matched pairs
        rotation = self.compute_z_rotation_from_vectors(
            gps_vectors, gps_distances,
            vio_vectors, vio_distances
        )
        
        if rotation is None:
            self.get_logger().warn('Could not compute rotation')
            return
        
        # Step 5: Apply rotation to ENTIRE VIO path
        aligned_path = self.apply_z_rotation_to_path(self.latest_vio_path, rotation)
        
        if aligned_path is not None:
            aligned_path.header.stamp = self.get_clock().now().to_msg()
            self.aligned_path_pub.publish(aligned_path)
            self.get_logger().info(f'Published aligned path with {len(aligned_path.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = PathAlignmentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()