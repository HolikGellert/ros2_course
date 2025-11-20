import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import numpy as np
import time

from action_interfaces.action import Grasp

class GraspServer(Node):

    def __init__(self):
        super().__init__('grasp_server')
        # Robot állapot feliratkozások
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

        # Robot parancs publisherek
        self.servo_cp_pub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.jaw_pub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            Grasp,
            'grasp',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Grasp action server indítva, várakozik a célokra...')

    def cb_measured_cp(self, msg):
        self.measured_cp = msg

    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg

    def goal_callback(self, goal_request):
        # Elfogadjuk az új célt
        self.get_logger().info('Új cél érkezett, elfogadva.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Elfogadjuk a megszakítási kérelmet
        self.get_logger().info('Cél megszakítása kérés érkezett.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Cél végrehajtása...')

        feedback_msg = Grasp.Feedback()
        result = Grasp.Result()

        # Konstansok (ezek korábban a main-ben voltak)
        v = 0.01
        omega = 0.1
        dt = 0.01

        # --- 1. Gripper nyitása ---
        feedback_msg.status = 'Gripper nyitása...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)
        self.move_jaw_to(target=0.8, omega=omega, dt=dt)

        # Ellenőrizzük, hogy törölték-e a célt
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Cél megszakítva a gripper nyitása közben.'
            return result

        # --- 2. Mozgás a célpozícióba (JAVÍTOTT LOGIKA) ---
        feedback_msg.status = 'Mozgás a célpozícióba...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        # Változók a koordináta-transzformációhoz
        target_pose = goal_handle.request.target_pose
        target_cam = np.array([target_pose.position.x,
                               target_pose.position.y,
                               target_pose.position.z])

        t_base_cam = np.array([0.18, 0.03, 0.01])
        R_base_cam = np.array([[-0.57922797, -0.27504447, -0.76736269],
                               [-0.40557979,  0.9138093,  -0.02139151],
                               [ 0.70710678,  0.29883624, -0.64085638]])

        # Transzformáció és offset (pre-grasp pozíció)
        target_base = (R_base_cam @ target_cam) + t_base_cam
        target_base = target_base + np.array([0.0, 0.0, 0.008])

        # Mozgás a kiszámított célponthoz
        self.move_tcp_to(target=target_base, v=v, dt=dt)

        # Ellenőrizzük, hogy törölték-e a célt
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Cél megszakítva a mozgás közben.'
            return result

        # --- 3. Gripper zárása ---
        feedback_msg.status = 'Gripper zárása...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)
        self.move_jaw_to(target=0.0, omega=omega, dt=dt)

        # --- Befejezés ---
        goal_handle.succeed()
        result.success = True
        result.message = 'Megfogás sikeres!'
        self.get_logger().info(result.message)
        return result

    # --- Mozgató segédfüggvények (az eredeti szkriptből átemelve) ---

    def move_tcp_to(self, target, v, dt):
        loop_rate = self.create_rate(100, self.get_clock())
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Várakozás a /PSM1/measured_cp-re...')
            rclpy.spin_once(self)
            loop_rate.sleep()

        # Reset rate for movement
        loop_rate = self.create_rate(1.0/dt, self.get_clock())
        pos_curr_np = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z])
        pos_target_np = np.array(target)
        distance = np.linalg.norm(pos_target_np - pos_curr_np)

        # Elkerüljük a nullával való osztást
        if v == 0:
            self.get_logger().warn("A sebesség (v) nulla, a mozgás nem lehetséges.")
            return

        T = distance / v
        N = int(round(abs(T / dt)))

        if N == 0: # Ha már a célponton vagyunk
            return

        tr_x = np.linspace(start=self.measured_cp.pose.position.x, stop=target[0], num=N)
        tr_y = np.linspace(start=self.measured_cp.pose.position.y, stop=target[1], num=N)
        tr_z = np.linspace(start=self.measured_cp.pose.position.z, stop=target[2], num=N)

        # Ahelyett, hogy a self.measured_cp-t módosítanánk, hozzunk létre egy új üzenetet
        # Ez biztonságosabb, és elkerüli a race condition-öket a callback-kel
        msg_template = self.measured_cp

        for i in range(N):
            if not rclpy.ok():
                break

            # Új üzenet létrehozása minden lépésben
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = msg_template.header.frame_id
            msg.pose.position.x = tr_x[i]
            msg.pose.position.y = tr_y[i]
            msg.pose.position.z = tr_z[i]
            # Tartsuk meg az eredeti orientációt
            msg.pose.orientation = msg_template.pose.orientation

            self.servo_cp_pub.publish(msg)
            #rclpy.spin_once(self) # Spin_once helyett sleep-et használunk a rate-tel
            loop_rate.sleep()

        # Kis várakozás, hogy a robot biztosan elérje a célpozíciót
        time.sleep(0.1)


    def move_jaw_to(self, target, omega, dt):
        loop_rate = self.create_rate(100, self.get_clock())
        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Várakozás a /PSM1/jaw/measured_js-re...')
            rclpy.spin_once(self)
            loop_rate.sleep()

        loop_rate = self.create_rate(1.0/dt, self.get_clock())
        distance = self.measured_jaw.position[0] - target

        if omega == 0:
            self.get_logger().warn("A jaw sebesség (omega) nulla, a mozgás nem lehetséges.")
            return

        T = abs(distance / omega)
        N = int(round(abs(T / dt)))

        if N == 0: # Ha már a célponton vagyunk
            return

        tr_jaw = np.linspace(start = self.measured_jaw.position[0], stop = target, num = N)

        msg_template = self.measured_jaw

        for i in range(N):
            if not rclpy.ok():
                break

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Másoljuk az attribútumokat a sablonból
            msg.name = msg_template.name
            msg.velocity = msg_template.velocity
            msg.effort = msg_template.effort

            # Állítsuk be az új pozíciót
            msg.position = [tr_jaw[i]]

            self.jaw_pub.publish(msg)
            #rclpy.spin_once(self)
            loop_rate.sleep()

        # Kis várakozás, hogy a robot biztosan elérje a célpozíciót
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    grasp_server = GraspServer()

    # Az eredeti main-ben volt egy reset mozgás. Ezt most a szerver indításakor
    # megtehetjük, vagy egy külön "init" scriptbe helyezhetjük.
    # A példa kedvéért tegyük meg itt, mielőtt a spin-re váltunk.
    grasp_server.get_logger().info('Robot resetelése (alaphelyzetbe állítás)...')
    grasp_server.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    grasp_server.move_jaw_to(0.0, 0.1, 0.01)
    grasp_server.get_logger().info('Reset kész. Várakozás a célokra.')

    try:
        rclpy.spin(grasp_server)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
