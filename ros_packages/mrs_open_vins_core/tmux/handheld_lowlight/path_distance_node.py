#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math
import time


class PathDistanceNode(Node):
    def __init__(self):
        super().__init__('path_distance_node')
        
        # Create subscription to Path topic
        self.subscription = self.create_subscription(
            Path,
            '/uav1/open_vins/pathimu',
            self.path_callback,
            1
        )
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('PathDistanceNode started, listening to /path topic')
    
    def path_callback(self, msg: Path):
        """Callback function when a Path message is received"""
        if len(msg.poses) < 2:
            self.get_logger().warn('Path has less than 2 poses, cannot compute distance')
            return
        
        # Get start and end points
        start = msg.poses[0].pose.position
        end = msg.poses[-1].pose.position
        
        # Compute Euclidean distance
        distance = math.sqrt(
            (end.x - start.x)**2 +
            (end.y - start.y)**2 +
            (end.z - start.z)**2
        )

        step = 5
        if len(msg.poses) > step:
            total_length = 0
            for i in range(step+1,len(msg.poses), step):
                total_length += math.sqrt(
                    (msg.poses[i].pose.position.x - msg.poses[i-step].pose.position.x)**2 +
                    (msg.poses[i].pose.position.y - msg.poses[i-step].pose.position.y)**2 +
                    (msg.poses[i].pose.position.z - msg.poses[i-step].pose.position.z)**2
                )

            total_length += math.sqrt(
                (msg.poses[-1].pose.position.x - msg.poses[-step].pose.position.x)**2 +
                (msg.poses[-1].pose.position.y - msg.poses[-step].pose.position.y)**2 +
                (msg.poses[-1].pose.position.z - msg.poses[-step].pose.position.z)**2
            )

            if total_length > 0:
                self.get_logger().info(
                    f'Endpoints distance: {distance:.2f}m\n'
                    f'Total length: {total_length:.2f}m\n'
                    f'Endpoints distance / Total length: {100*(distance / total_length):.2f}%\n'
                )
        
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = PathDistanceNode()
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
