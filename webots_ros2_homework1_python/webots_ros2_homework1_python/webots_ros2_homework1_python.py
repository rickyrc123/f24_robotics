import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan, NavSatFix
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import random



LINEAR_VEL = 0.22
TARGET_DIST = 1.0


STOP_DISTANCE = 0.35
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
WINDOW_SIZE = 0.05

MIN_DIST_INDEX = 0
MOVEMENT_LOG= []
MAX_DIST_LOG = []
STEPS_PER_DRIVE = 5
TOTAL_DISTANCE = 0

TIMER_PERIOD = 0.1


class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.subscriber_gps = self.create_subscription(NavSatFix, '/TurtleBot3Burger/gps', self.gps_callback, 10)

        self.current_dist = 0.0
        self.start_pos_odom = None
        self.start_pos_gps = None
        self.odom_pos = None
        self.gps_pos = None

        self.timer_period = TIMER_PERIOD
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cmd = Twist()
        self.moving = True


    def odom_callback(self, msg):
        position = msg.pose.pose.position

        if self.start_pos_odom is None:
            self.start_pos_odom = position
        
        self.odom_pos = position

        self.current_dist = self.calculate_distance(self.start_pos_odom, position)

    
    # def gps_callback(self, msg):
    #     if self.start_pos_gps is None:
    #         self.start_pos_gps = (msg.x, msg.y)
        
    #     self.gps_pos = (msg.x, msg.y)


    def calculate_distance(self, start, current):
        dx = current.x - start.x
        dy = current.y - start.y
        return math.sqrt(dx**2 + dy**2)

    
    # def calculate_gps_distance(self, start, current):
    #     dx = current(0) - start(0)
    #     dy = current(1) - start(1)
    #     return math.sqrt(dx**2 + dy**2)
  
    def timer_callback(self):
        if self.moving == True:
            if self.current_dist < TARGET_DIST:
                self.cmd.linear.x = LINEAR_VEL
            else:
                self.cmd.linear.x = 0.0
                self.moving = False
                self.get_logger().info(f'Reached {TARGET_DIST} m')
            
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Odometry pose: {self.odom_pos}')



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
