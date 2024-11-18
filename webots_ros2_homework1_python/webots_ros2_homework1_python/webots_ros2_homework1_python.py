import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from apritag_msgs.msg import ApritagDetectionArray
from geometry_msgs.msg import Twist
from time import time

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.april_sub = self.create_subscription(ApritagDetectionArray, '/apriltag_detections', self.april_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_radius = 0.2
        self.safe_distance = 0.5
        self.frontiers = []
        self.current_goal = None
        self.goal_reached = False
        self.robot_position = (0.0, 0.0)
        self.goal_tolerance = 0.3
        self.laser_scan = None
        self.last_spin_time = time()
        self.is_spinning = False
        self.spin_start_time = None
        self.detected_tags = set()

        self.exploration_timer = self.create_timer(1.0, self.explore)


    def map_callback(self, msg):
        map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.frontiers = self.find_frontiers(map_data, resolution, origin_x, origin_y)
    
    def odom_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def scan_callback(self, msg):
        self.laser_scan = msg

    def april_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.detected_tags:
                self.detected_tags.add(tag_id)
                self.get_logger().info(f'Detected tag {tag_id}')

    def find_frontiers(self, map_data, resolution, origin_x, origin_y):
        frontiers = []
        for i in range(1, map_data.shape[0]-1):
            for j in range(1, map_data.shape[1]-1):
                if map_data[i, j] == 0:
                    if np.any(map_data[i-1:i+2, j-1:j+2] == -1):
                        x = origin_x + j * resolution
                        y = origin_y + i * resolution
                        frontiers.append((x, y))
        return frontiers
    
    def explore(self):
        current_time = time()
        if current_time - self.last_spin_time >= 10 and not self.is_spinning:
            self.is_spinning = True
            self.spin_start_time = current_time
            self.get_logger().info('Spinning')
            self.perform_spin()
            return

        if self.is_spinning:
            if current_time - self.spin_start_time < 5:
                self.perform_spin()
            else:
                self.is_spinning = False
                self.last_spin_time = current_time
                self.get_logger().info('Done spinning')
            return
        
        if not self.frontiers:
            self.get_logger().info('No frontiers found')
            return

        if self.current_goal is None or self.goal_reached:
            self.current_goal = self.select_nearest_frontier()
            if self.current_goal:
                self.goal_reached = False
                self.navigate_to_goal(self.current_goal)
        else:
            distance_to_goal = self.calculate_distance(self.robot_position, self.current_goal)
            if distance_to_goal < self.goal_tolerance:
                self.get_logger().info('Goal reached')
                self.goal_reached = True
            else:
                self.navigate_to_goal(self.current_goal)
            
    
    def select_nearest_frontier(self):
        min_distance = float('inf')
        nearest_frontier = None
        for x, y in self.frontiers:
            distance = self.calculate_distance(self.robot_position, (x, y))
            if distance < min_distance:
                min_distance = distance
                nearest_frontier = (x, y)
        return nearest_frontier
    
    def calculate_distance(self, position1, position2):
        return np.sqrt((position1[0] - position2[0])**2 + (position1[1] - position2[1])**2)
    
    def navigate_to_goal(self, goal):
        goal_x, goal_y = goal
        robt_x, robot_y = self.robot_position
        angle_to_goal = np.arctan2(goal_y - robot_y, goal_x - robt_x)
        distance_to_goal = self.calculate_distance((robot_x, robot_y), (goal_x, goal_y))

        twist = Twist()
        if distance_to_goal > self.goal_tolerance:
            twist.linear.x = 0.2
            twist.angular.x = angle_to_goal
        else:
            twist.linear.x = 0.0
            twist.angular.x = 0.0
            self.goal_reached = True
            self.get_logger().info('Goal reached')

        self.cmd_pub.publish(twist)

    def perform_spin(self):
        twist = Twist()
        twist.angular.z = 1.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()            