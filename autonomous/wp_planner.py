import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('WaypointPlanner')

        # 1. Load Waypoints from file
        self.waypoints = self.load_waypoints('waypoints.txt')
        self.current_wp_idx = 0
        
        # 2. State variables
        self.current_position = None
        self.target_accuracy = 0.5  # Distance threshold to waypoint

        # Publishers & Subscribers
        # Restored to PointStamped as per your request
        self.wp_publisher = self.create_publisher(PointStamped, '/way_point', 10)
        
        # Odometry Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            # '/odom',
            '/dlio/odom_node/map_pose',
            self.odom_callback,
            10
        )

        # Create timer (200 Hz)
        self.timer = self.create_timer(0.005, self.timer_callback)

        self.get_logger().info('Waypoint Planner Node (PointStamped Mode) has started.')
    
    def load_waypoints(self, filename):
        """Loads x,y,z waypoints from a text file."""
        waypoints = []
        if not os.path.exists(filename):
            self.get_logger().warn(f"File {filename} not found! Creating a dummy waypoint list.")
            return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
            
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    coords = [float(x) for x in line.split(',')]
                    if len(coords) == 3:
                        waypoints.append(coords)
                except ValueError:
                    self.get_logger().error(f"Could not parse line: {line}")
                    
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints successfully.")
        return np.array(waypoints)

    def odom_callback(self, msg):
        """Updates current position and checks distance to target waypoint."""
        # Extract current position
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])
        
        if len(self.waypoints) == 0 or self.current_wp_idx >= len(self.waypoints):
            return

        # Target waypoint
        target_wp = self.waypoints[self.current_wp_idx]
        
        # Compute Euclidean distance
        distance = np.linalg.norm(self.current_position - target_wp)
        
        # Check if we reached the waypoint
        if distance < self.target_accuracy:
            self.get_logger().info(f"Reached waypoint {self.current_wp_idx}: {target_wp}")
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints accomplished!")

    def timer_callback(self):
        # If we have run out of waypoints, do nothing
        if len(self.waypoints) == 0 or self.current_wp_idx >= len(self.waypoints):
            return

        # Fetch active target coordinates
        target_wp = self.waypoints[self.current_wp_idx]

        # Construct PointStamped message
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = ''
        
        point_msg.point.x = float(target_wp[0])
        point_msg.point.y = float(target_wp[1])
        point_msg.point.z = float(target_wp[2])

        # Publish target point directly
        self.wp_publisher.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()