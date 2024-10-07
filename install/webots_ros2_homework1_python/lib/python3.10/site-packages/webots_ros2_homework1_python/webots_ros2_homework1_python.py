import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
MAX_SPEED = 0.3

DESIRED_ANGLE = 10
STOP_ANGLE = 5
MAX_TURN = 0.5235988

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.laser_forward = 0
        self.odom_data = 0
        self.pose_saved = ''
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.angle = 0
        self.prev_angle = 0
        self.total_angle = 0
        self.ready = False
        self.start = 0

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z * 180, orientation.w)
        
        if not self.ready:
            self.start = qz
            self.ready = True
        
        self.pose_saved = position
        self.angle = qz - self.start

    def timer_callback(self): 
        if self.ready:       
            self.total_angle += math.fabs(self.angle - self.prev_angle)
            self.prev_angle = self.angle
            angle = self.total_angle
            
            self.get_logger().info('Angle: {} Msg: {} Prev: {}'.format(angle, self.angle, self.prev_angle))

            if angle < DESIRED_ANGLE:
                if angle < DESIRED_ANGLE - STOP_ANGLE:
                    self.cmd.angular.z = MAX_TURN
                else:
                    turn = max(0.15, MAX_TURN * (STOP_ANGLE - (angle - (DESIRED_ANGLE - STOP_ANGLE))) / STOP_ANGLE)
                    self.cmd.angular.z = turn
            else:
                self.cmd.angular.z = 0.0

            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    try:
        rclpy.spin(random_walk_node)
    except KeyboardInterrupt:
        random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
