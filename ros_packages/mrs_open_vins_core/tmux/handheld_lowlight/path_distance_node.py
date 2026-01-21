#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import math
import time
import matplotlib.pyplot as plt
import numpy as np


class PathDistanceNode(Node):
    def __init__(self):
        super().__init__('path_distance_node')

        self.declare_parameter("path_topic", "")
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        #path_topic = self.declare_parameter("path_topic", "").value
        self.get_logger().info(f'PathDistanceNode started, listening to "{path_topic}" topic')
        
        # Create subscription to Path topic
        self.path_subscription = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            1
        )
        self.path_subscription  # prevent unused variable warning

        self.path = None

        # Create subscription to /finish topic, which triggers the processing ofthe results
        self.finish_subscription = self.create_subscription(
            Bool,
            '/finish',
            self.finish_callback,
            1
        )
        self.finish_subscription  # prevent unused variable warning

    def finish_callback(self, msg: Bool):
        if not msg.data:
            print("Finish was not succesful, not processing anyting")

        if self.path is None:
            print("Something went wrong, path is empty")
        else:
            if len(self.path.poses) < 2:
                self.get_logger().warn('Path has less than 2 poses, cannot compute distance')
                return
            
            # Get start and end points
            start = self.path.poses[0].pose.position
            end = self.path.poses[-1].pose.position
            
            # Compute Euclidean distance
            distance = math.sqrt(
                (end.x - start.x)**2 +
                (end.y - start.y)**2 +
                (end.z - start.z)**2
            )

            step = 5
            if len(self.path.poses) > step:
                total_length = 0
                velocities = []
                segment_distances = []
                segment_times = []
                
                for i in range(step+1, len(self.path.poses), step):
                    # Compute distance for this segment
                    segment_distance = math.sqrt(
                        (self.path.poses[i].pose.position.x - self.path.poses[i-step].pose.position.x)**2 +
                        (self.path.poses[i].pose.position.y - self.path.poses[i-step].pose.position.y)**2 + 0
                        #(self.path.poses[i].pose.position.z - self.path.poses[i-step].pose.position.z)**2
                    )
                    total_length += segment_distance
                    segment_distances.append(segment_distance)
                    
                    # Compute time duration for this segment
                    time_start = self.path.poses[i-step].header.stamp.sec + self.path.poses[i-step].header.stamp.nanosec / 1e9
                    time_end = self.path.poses[i].header.stamp.sec + self.path.poses[i].header.stamp.nanosec / 1e9
                    delta_time = time_end - time_start
                    segment_times.append(delta_time)
                    
                    # Compute velocity for this segment
                    if delta_time > 0:
                        velocity = segment_distance / delta_time
                        velocities.append(velocity)
                    else:
                        self.get_logger().warn(f'Segment {len(velocities)} has zero or negative time duration')
                        velocities.append(0)

                # Handle final segment
                final_segment_distance = math.sqrt(
                    (self.path.poses[-1].pose.position.x - self.path.poses[-step].pose.position.x)**2 +
                    (self.path.poses[-1].pose.position.y - self.path.poses[-step].pose.position.y)**2 + 0
                    #(self.path.poses[-1].pose.position.z - self.path.poses[-step].pose.position.z)**2
                )
                total_length += final_segment_distance
                segment_distances.append(final_segment_distance)
                
                time_start_final = self.path.poses[-step].header.stamp.sec + self.path.poses[-step].header.stamp.nanosec / 1e9
                time_end_final = self.path.poses[-1].header.stamp.sec + self.path.poses[-1].header.stamp.nanosec / 1e9
                delta_time_final = time_end_final - time_start_final
                segment_times.append(delta_time_final)
                
                if delta_time_final > 0:
                    velocity_final = final_segment_distance / delta_time_final
                    velocities.append(velocity_final)
                else:
                    self.get_logger().warn('Final segment has zero or negative time duration')
                    velocities.append(0)

                if total_length > 0:
                    # Compute velocity statistics
                    velocities_array = np.array(velocities)
                    mean_velocity = np.mean(velocities_array)
                    max_velocity = np.max(velocities_array)
                    min_velocity = np.min(velocities_array)
                    
                    self.get_logger().info(
                        f'Stats:\n'
                        f'\tTotal num. of poses: {len(self.path.poses)}\n'
                        f'\tEndpoints distance: {distance:.2f}m\n'
                        f'\tTotal length: {total_length:.2f}m\n'
                        f'\tEndpoints distance / Total length: {100*(distance / total_length):.2f}%\n'
                        f'\tMean velocity: {mean_velocity:.3f} m/s\n'
                        f'\tMax velocity: {max_velocity:.3f} m/s\n'
                        f'\tMin velocity: {min_velocity:.3f} m/s\n'
                    )
                    
                    # Plot velocity
                    #self._plot_velocity(velocities, segment_distances, segment_times)
        
    def _plot_velocity(self, velocities, segment_distances, segment_times):
        """Plot velocity profile over the path"""
        try:
            segment_indices = np.arange(len(velocities))
            
            fig, axes = plt.subplots(1, 1, figsize=(12, 10))

            axes.plot(segment_indices, velocities, marker='o', linestyle='-', linewidth=2)
            axes.set_xlabel('Segment Number')
            axes.set_ylabel('Velocity (m/s)')
            axes.set_title('Velocity Profile Over Path Segments')
            axes.grid(True, alpha=0.3)
            
            # Plot 1: Velocity over segments
            # axes[0].plot(segment_indices, velocities, marker='o', linestyle='-', linewidth=2)
            # axes[0].set_xlabel('Segment Number')
            # axes[0].set_ylabel('Velocity (m/s)')
            # axes[0].set_title('Velocity Profile Over Path Segments')
            # axes[0].grid(True, alpha=0.3)
            
            # # Plot 2: Segment distances
            # axes[1].bar(segment_indices, segment_distances, color='steelblue', alpha=0.7)
            # axes[1].set_xlabel('Segment Number')
            # axes[1].set_ylabel('Distance (m)')
            # axes[1].set_title('Distance Per Segment')
            # axes[1].grid(True, alpha=0.3, axis='y')
            
            # # Plot 3: Segment times
            # axes[2].bar(segment_indices, segment_times, color='coral', alpha=0.7)
            # axes[2].set_xlabel('Segment Number')
            # axes[2].set_ylabel('Time (s)')
            # axes[2].set_title('Time Duration Per Segment')
            # axes[2].grid(True, alpha=0.3, axis='y')
            
            plt.tight_layout()
            plt.savefig('/tmp/path_velocity_analysis.png', dpi=150)
            self.get_logger().info('Velocity analysis plot saved to /tmp/path_velocity_analysis.png')
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Failed to plot velocity: {e}')
        
    def path_callback(self, msg: Path):
        """Callback function when a Path message is received"""
        self.path = msg
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = PathDistanceNode()
    
    # Spin for a limited time or until the node is done
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()