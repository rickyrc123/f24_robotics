import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
import csv

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
WALL_FOLLOW_DISTANCE = SAFE_STOP_DISTANCE + 0.22 #how far should the bot be from the wall

class RightWallFollow(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False    #is robot stalled?
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=None #compare position for stalls
        self.pose_curr=None  #current position
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #wall following
        self.following_wall = False   #is robot on track with wall?
        self.begin_time = time.time()
        self.following_wall_time = 0.0   #last time robot found wall

        #stall detection
        self.stall_time = 0.0    #track how long robot hasnt been moving
        self.stall_time_threshold = 5.0 #how long until no movement is considered a stall
        self.stall_detected_time = None #time when stall occurs
        self.recovery = False #is robot recovering from stall

    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif (math.isnan(reading) or reading == 0.0):
                print ('nan or 0')
                #self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        self.pose_curr = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        #(posx, posy, posz) = (position.x, position.y, position.z)
        #(qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        #self.pose_saved=position

        # Check if position has changed 
        if self.pose_saved is not None:
            diffX = math.fabs(self.pose_saved.x - self.pose_curr.x)
            diffY = math.fabs(self.pose_saved.y - self.pose_curr.y)
            

        # Checking position differences
            if diffX < 0.0001 and diffY < 0.0001:  #robot hasnt moved significantly
                if self.stall_detected_time == None:
                    self.stall_detected_time = time.time()
                self.stall_time = time.time() - self.stall_detected_time
                #self.stall = True
            else:
                self.stall_detected_time = None
                self.stall_time = 0.0
                #self.stall = False

        # Update the saved position
        self.pose_saved = self.pose_curr
 
    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        #lidar measurements
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        #write positions to text file and terminal
        with open('positions_log.txt', 'a') as f:
            # print position to terminal
            #self.get_logger().info(f'Current position: {self.pose_curr}')
            
            # Print just x and y values to text file
            f.write(f'{self.pose_curr.x}, {self.pose_curr.y}\n')
        
        #get current time
        curr_time = time.time()        

        #is robot currently following the right wall?
        if right_lidar_min > WALL_FOLLOW_DISTANCE + 0.08 and (curr_time - self.following_wall_time > 6.0):     #how long since its followed the wall?
            self.following_wall = False #hasn't been following wall for a considerable amt of time

        #is the robot stalled?
        if self.stall_time > self.stall_time_threshold:
            self.get_logger().info('Stall detected!')
            self.cmd.linear.x = -LINEAR_VEL #straight backup out of stall
            self.cmd.angular.z = 0.0
            self.recovery = True
            self.stall_time = 0.0
        elif self.recovery:
            self.get_logger().info('Stall recovery')
            #find best path out of stall
            self.cmd.linear.x = 0.0 #stop moving
            if left_lidar_min > right_lidar_min:
                self.cmd.angular.z = 0.4 #more room on left, turn there
            else:
                self.cmd.angular.z = -0.4 #more room on right, turn there
            self.recovery = False
            self.stall_time = 0.0
            self.stall_detected_time = 0.0
            self.stall = False
        # Is there is an obstacle in front?
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.08  # slow down moving forward
            #can follow the right wall?
            if right_lidar_min > WALL_FOLLOW_DISTANCE and self.following_wall:
                    self.cmd.angular.z = -0.4  # currently close to wall, turn right to avoid the front obstacle
                    self.following_wall_time = curr_time # currently following wall
            else:
                self.cmd.angular.z = 0.4 #currently not following right wall, turn left to avoid obstacle
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Object in front, turning away')
        else:   # No object in front, right wall following logic
            if right_lidar_min < SAFE_STOP_DISTANCE:   # Too close to the right wall, turn left
                self.cmd.linear.x = 0.08
                self.cmd.angular.z = 0.11  # Turn left
                self.following_wall = True
                self.following_wall_time = curr_time
                self.get_logger().info('Too close to right wall, turning left')
            elif right_lidar_min > WALL_FOLLOW_DISTANCE:  # Too far from the right wall, turn right
                self.cmd.linear.x = 0.08
                self.cmd.angular.z = -0.19  # Turn right
                self.get_logger().info('Finding right wall, turning right')
            else:   # Right wall is at a good distance, move forward
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0
                self.get_logger().info('Following right wall')
                self.following_wall = True
                self.following_wall_time = curr_time

        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = True


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RightWallFollow()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
