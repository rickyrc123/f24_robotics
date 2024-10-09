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

#10 - 0.174
#30 - 0.523
#120 - 2.094
#180 - 3.141
#359 - 6.26

LINEAR_VEL = 0.15
TARGET_DIST = 5.0

ROTATIONAL_VEL = 0.523
TARGET_ANGLE = 6.26
WRAP_TOLERANCE = 0.0087


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
        self.start_theta = None 
        self.current_theta = None

        self.timer_period = TIMER_PERIOD
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cmd = Twist()
        self.moving = False
        self.turning = True

        self.angle_to_turn = None
        self.kickstart = True
        self.rotation_count = 0
        self.transition_flag = False
        self.last_theta = None
        self.total_rotation = 0


    def odom_callback(self, msg):
        #position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # if self.start_pos_odom is None:
        #     self.start_pos_odom = position

        if self.start_theta is None:
            self.start_theta = self.get_yaw(orientation)

        if self.angle_to_turn is None:
            self.angle_to_turn = 2*math.pi - TARGET_ANGLE
            
        
        #self.odom_pos = position
        if self.turning is True:
            self.current_theta = self.get_yaw(orientation)

        

        #self.current_dist = self.calculate_distance(self.start_pos_odom, position)

    
    # def gps_callback(self, msg):
    #     if self.start_pos_gps is None:
    #         self.start_pos_gps = (msg.x, msg.y)
        
    #     self.gps_pos = (msg.x, msg.y)


    def calculate_distance(self, start, current):
        dx = current.x - start.x
        dy = current.y - start.y
        return math.sqrt(dx**2 + dy**2)
    
    def get_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        calced = math.atan2(siny_cosp, cosy_cosp)
        # if calced:
        #     return 2*math.pi + calced
        # else:
        #     #self.get_logger().info(f'angle in calc: {calced}')
        #     return calced
        return calced
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def within_tolerance(self, angle):
        if angle < (0 + WRAP_TOLERANCE):
            return True
        else:
            return False
        
    def udpate_total_rotation(self, angle):
        if self.last_theta is None:
            self.last_theta = angle
            return self.total_rotation
        
        delta = angle - self.last_theta

        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2* math.pi

        self.total_rotation += delta
        self.last_theta = angle
    
    # def calculate_gps_distance(self, start, current):
    #     dx = current(0) - start(0)
    #     dy = current(1) - start(1)
    #     return math.sqrt(dx**2 + dy**2)
  
    def timer_callback(self):
        # if self.moving == True:
        #     if self.current_dist < TARGET_DIST:
        #         self.cmd.linear.x = LINEAR_VEL
        #     else:
        #         self.cmd.linear.x = 0.0
        #         self.moving = False
        #         self.get_logger().info(f'Reached {TARGET_DIST} m')
            
            

        if self.turning == True and self.start_theta is not None:

            if self.kickstart == True:
                self.cmd.angular.z = ROTATIONAL_VEL
                self.kickstart = False
            
            # if self.current_theta < 0:
            #     self.current_theta = 2 * math.pi + self.current_theta

            # self.current_theta += math.pi * self.rotation_count
                

            # if self.within_tolerance(self.current_theta) and self.last_theta > 0:
            #     self.rotation_count += 1

                
            self.udpate_total_rotation(self.current_theta)
            
            
            
            if  abs(self.total_rotation) < TARGET_ANGLE:
                self.cmd.angular.z = ROTATIONAL_VEL
            else:
                self.cmd.angular.z = 0.0
                self.turning = False
                self.get_logger().info(f'Reached {math.degrees(TARGET_ANGLE)} deegres')
                
            self.last_theta = self.current_theta
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Theta: {math.degrees(self.current_theta)}, Last Theta: {self.last_theta}, Rotation Count: {self.rotation_count}, Total Rotation: {math.degrees(self.total_rotation)}')
         #  Odometry pose: {self.odom_pos}

            



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
