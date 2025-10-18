import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np


class PSM(Node):

    def __init__(self):
        super().__init__('psm_grasp')
        # Subscribers
        self.measured_cp = None
        self.subscription_measured_cp = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10)

        self.measured_jaw = None
        self.subscription_measured_jaw = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_measured_jaw,
            10)

        self.marker = None
        self.subscription_marker = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.cb_marker,
            10)

        # Publishers
        self.servo_cp_pub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.jaw_pub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)


    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        #print(self.measured_jaw)

    def cb_marker(self, msg):
        self.marker = msg
        #print(self.measured_jaw)

    def grasp_marker(self, v, omega, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.marker is None and rclpy.ok():
            self.get_logger().info('Waiting for marker pos...')
            rclpy.spin_once(self)

        self.get_logger().info('Opening gripper...')
        self.move_jaw_to(target = 0.8, omega = omega, dt = dt)

        self.get_logger().info('Moving to target...')
        target = np.array([ self.marker.pose.position.x,
                            self.marker.pose.position.y,
                            self.marker.pose.position.z])

        t_base_cam = np.array([0.18, 0.03, 0.01])
        R_base_cam = np.array([[-0.57922797, -0.27504447, -0.76736269],
                         [-0.40557979,  0.9138093,  -0.02139151],
                         [ 0.70710678,  0.29883624, -0.64085638]])

        target_base = (R_base_cam @ target_cam) + t_base_cam

        target_base = target_base + np.array([0.0, 0.0, 0.008])

        self.move_tcp_to(target = target, v = 0.01, dt = 0.01)

        self.get_logger().info('Closing gripper...')
        self.move_jaw_to(target = 0.0, omega = omega, dt = dt)


    def move_tcp_to(self, target, v, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)
        loop_rate = self.create_rate(1.0/dt, self.get_clock()) # Hz
        pos_curr_np = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z])
        pos_target_np = np.array(target)
        distance = np.linalg.norm(pos_target_np - pos_curr_np)
        T = distance / v
        N = int(round(abs(T / dt)))
        tr_x = np.linspace(start=self.measured_cp.pose.position.x, stop=target[0], num=N)
        tr_y = np.linspace(start=self.measured_cp.pose.position.y, stop=target[1], num=N)
        tr_z = np.linspace(start=self.measured_cp.pose.position.z, stop=target[2], num=N)
        for i in range(len(tr_x)):
            if not rclpy.ok():
                break
            msg = self.measured_cp
            msg.pose.position.x = tr_x[i]
            msg.pose.position.y = tr_y[i]
            msg.pose.position.z = tr_z[i]
            self.servo_cp_pub.publish(msg)
            rclpy.spin_once(self)

    def move_jaw_to(self, target, omega, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Waiting for jaw pos...')
            rclpy.spin_once(self)

        loop_rate = self.create_rate(1.0/dt, self.get_clock()) # Hz
        distance = self.measured_jaw.position[0] - target
        T = abs(distance / omega)
        N = int(round(abs(T / dt)))
        tr_jaw = np.linspace(start = self.measured_jaw.position[0], stop = target, num = N)

        for i in range(len(tr_jaw)):
            if not rclpy.ok():
                break
            msg = self.measured_jaw
            msg.position = [tr_jaw[i]]
            self.jaw_pub.publish(msg)
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    psm = PSM()
    #Reset the arm
    psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    psm.move_jaw_to(0.0, 0.1, 0.01)

    psm.grasp_marker(v = 0.01, omega = 0.1, dt = 0.01)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
