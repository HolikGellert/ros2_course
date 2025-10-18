import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)


    # New method for TurtlesimController
    def cb_pose(self, msg):
        self.pose = msg


    def go_to(self, speed, omega, x, y):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        #Stuff with atan2

        theta_0 = self.pose.theta
        pos_0 = np.array([self.pose.x, self.pose.y])
        pos_1 = np.array([x, y])
        disp_vec = pos_1 - pos_0
        theta_1 = math.atan2(disp_vec[1], disp_vec[0])
        angle = math.degrees(theta_1 - theta_0)
        distance = np.linalg.norm(disp_vec)

        self.turn(omega=omega, angle=angle)
        self.go_straight(speed=speed, distance=distance)



    def turn(self, omega, angle):
        # Implement rotation here
        # Create and publish msg

        omega_rad = math.radians(omega)
        angle_rad = math.radians(angle)

        vel_msg = Twist()
        if angle_rad > 0:
            vel_msg.angular.z = omega_rad
        else:
            vel_msg.angular.z = -omega_rad
        vel_msg.angular.y = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(angle_rad / omega_rad)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when and rclpy.ok()):
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')


    def draw_square(self, speed, omega, a):
        # Implement
        for i in range(4):
            self.go_straight(speed, a)
            self.turn(omega, 90.0)


    def draw_poly(self, speed, omega, N, a):
        # Implement
        angle=360.0/N
        for i in range(N):
            self.go_straight(speed, a)
            self.turn(omega, angle)


    def go_straight(self, speed, distance):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(distance / speed)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when and rclpy.ok()):
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    tc.go_to(1.0, 20.0, 2, 8)
    tc.go_to(1.0, 20.0, 2, 2)
    tc.go_to(1.0, 20.0, 3, 4)
    tc.go_to(1.0, 20.0, 6, 2)


    #tc.draw_square(speed=1.0, omega=90.0, a=3)
    #tc.draw_poly(speed=1.0, omega=90.0, N=5, a=2)
    #tc.draw_poly(speed=1.0, omega=90.0, N=6, a=2)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
