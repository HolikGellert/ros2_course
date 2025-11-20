import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState
import kinpy as kp
from std_msgs.msg import String


class UR(Node):

    def __init__(self):
        super().__init__('ur_controller')

        self.set_joint_states_pub = self.create_publisher(
                                                JointState,
                                                'set_joint_states',
                                                10)

        self.joint_states_curr = None
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb_joint_states,
            10)


        # A szimulátor egy topicban publikálja a robotot leíró urdf-t.
        # Iratkozzunk fel erre a topic-ra.

        self.chain = None
        self.description_sub = self.create_subscription(
            String,
            '/robot_description_latch',
            self.cb_desc,
            10)


    # New method
    def cb_joint_states(self, msg):
        self.joint_states_curr = msg
        #print(self.joint_states_curr)

    # New method
    def cb_desc(self, msg):
        if self.chain is None:
            self.chain = kp.build_serial_chain_from_urdf(msg.data, 'tool0')
            print(self.chain.get_joint_parameter_names())
            print(self.chain)


    def to_configuration(self, q):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.joint_states_curr is None and rclpy.ok():
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self)

        msg = self.joint_states_curr
        msg.position = q
        self.set_joint_states_pub.publish(msg)


    def calc_kinematics(self):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.joint_states_curr is None and rclpy.ok():
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self)
        # Wait for robot description to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.chain is None and rclpy.ok():
            self.get_logger().info('Waiting for robot desc...')
            rclpy.spin_once(self)

        q = self.joint_states_curr.position
        p = self.chain.forward_kinematics(q)
        return p


    def tcp_to_position(self, r_des):
        k_1 = 0.001
        omega = np.array([0.0, 0.0, 0.0])
        threshold = 0.01
        delta_r = 100000.0
        r_des_np = np.array(r_des)

        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while np.linalg.norm(delta_r) > threshold and rclpy.ok():
            r_curr_np = np.array(self.calc_kinematics().pos)
            delta_r = r_des_np - r_curr_np
            k_1_delta_r = k_1 * delta_r
            r_omega = np.concatenate((k_1_delta_r, omega))
            #print(r_omega)

            J = kp.jacobian.calc_jacobian(self.chain, self.joint_states_curr.position)

            J_inv = np.linalg.pinv(J)
            #J_inv = J.transpose()

            delta_q = J_inv @ r_omega

            q_curr_np = np.array(self.joint_states_curr.position)
            q_new_np = q_curr_np + delta_q
            q_new = q_new_np.tolist()
            self.to_configuration(q_new)

            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    ur = UR()



    #q = [-1.28, 4.41, 1.54, -1.16, -1.56, 0.0]
    #ur.to_configuration(q)

    #print(ur.calc_kinematics())
    #rclpy.spin(ur)

    print(ur.tcp_to_position(r_des=(0.50, -0.60, 0.20)))


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ur.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
