import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState

# Feltételezzük, hogy a csomag neve ros2_course_msgs és az action neve Grasp
from ros2_course_msgs.action import Grasp

class GraspServer(Node):

    def __init__(self):
        super().__init__('grasp_server')

        # Callback group a párhuzamos futtatáshoz (fontos a blocking move függvények miatt)
        self.cb_group = ReentrantCallbackGroup()

        # Állapot változók
        self.measured_cp = None
        self.measured_jaw = None

        # 1. Robot feliratkozások (Subscribers)
        self.subscription_measured_cp = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10,
            callback_group=self.cb_group)

        self.subscription_measured_jaw = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_measured_jaw,
            10,
            callback_group=self.cb_group)

        # 2. Robot vezérlés (Publishers)
        self.servo_cp_pub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.jaw_pub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)

        # 3. Action Server inicializálás
        self._action_server = ActionServer(
            self,
            Grasp,
            'grasp',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.get_logger().info('Grasp Action Server elindult és vár a célokra...')

    # --- Robot Callback-ek ---
    def cb_measured_cp(self, msg):
        self.measured_cp = msg

    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg

    # --- Action Server Callback-ek ---
    def goal_callback(self, goal_request):
        self.get_logger().info('Új cél érkezett.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Megszakítási kérelem érkezett.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Cél végrehajtása elkezdődött...')

        feedback_msg = Grasp.Feedback()
        result = Grasp.Result()

        # Mozgatási paraméterek
        v = 0.01      # TCP sebesség
        omega = 0.1   # Pofa sebesség
        dt = 0.01     # Időlépés

        # --- 1. LÉPÉS: Gripper nyitása ---
        feedback_msg.status = '1. lépés: Gripper nyitása...'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)

        success = self.move_jaw_to(target=0.8, omega=omega, dt=dt)
        if not success or goal_handle.is_cancel_requested:
            return self.handle_failure(goal_handle, result, "Hiba vagy megszakítás a nyitásnál.")

        # --- 2. LÉPÉS: Pozíció számítása és mozgás ---
        feedback_msg.status = '2. lépés: Mozgás a célpozícióba...'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)

        # Célkoordináták kiolvasása (geometry_msgs/Point -> numpy)
        # A goal definícióban: geometry_msgs/Point grasp_pos
        pos_req = goal_handle.request.grasp_pos
        target_cam = np.array([pos_req.x, pos_req.y, pos_req.z])

        # Transzformáció (Camera -> Base)
        t_base_cam = np.array([0.18, 0.03, 0.01])
        R_base_cam = np.array([[-0.57922797, -0.27504447, -0.76736269],
                               [-0.40557979,  0.9138093,  -0.02139151],
                               [ 0.70710678,  0.29883624, -0.64085638]])

        target_base = (R_base_cam @ target_cam) + t_base_cam
        # Offset hozzáadása (pl. kicsit a marker fölé)
        target_base = target_base + np.array([0.0, 0.0, 0.008])

        success = self.move_tcp_to(target=target_base, v=v, dt=dt)
        if not success or goal_handle.is_cancel_requested:
            return self.handle_failure(goal_handle, result, "Hiba vagy megszakítás a pozícionálásnál.")

        # --- 3. LÉPÉS: Gripper zárása (megfogás) ---
        feedback_msg.status = '3. lépés: Gripper zárása...'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)

        success = self.move_jaw_to(target=0.0, omega=omega, dt=dt)
        if not success or goal_handle.is_cancel_requested:
            return self.handle_failure(goal_handle, result, "Hiba vagy megszakítás a zárásnál.")

        # --- SIKERES BEFEJEZÉS ---
        goal_handle.succeed()
        result.success = True
        # result.message = "Sikeres megfogás!" # Csak ha definiáltad a message-t az .action fájlban
        self.get_logger().info('Action sikeresen befejeződött.')
        return result

    def handle_failure(self, goal_handle, result, msg):
        """Segédfüggvény a hiba/cancel kezelésre"""
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().warn('Action megszakítva.')
        else:
            goal_handle.abort()
            self.get_logger().error(f'Action hiba: {msg}')

        result.success = False
        # result.message = msg
        return result

    # --- Mozgató függvények (Blocking) ---

    def move_tcp_to(self, target, v, dt):
        rate = self.create_rate(1.0/dt)

        # Várakozás az adatokra
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Várakozás a pozíció adatokra...', throttle_duration_sec=2.0)
            rate.sleep()

        if not rclpy.ok(): return False

        current_pos = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z])
        target_np = np.array(target)
        distance = np.linalg.norm(target_np - current_pos)

        if v <= 0: return False
        T = distance / v
        steps = int(T / dt)

        if steps == 0: return True

        # Interpoláció
        traj_x = np.linspace(current_pos[0], target[0], steps)
        traj_y = np.linspace(current_pos[1], target[1], steps)
        traj_z = np.linspace(current_pos[2], target[2], steps)

        msg = PoseStamped()
        msg.header.frame_id = self.measured_cp.header.frame_id
        msg.pose.orientation = self.measured_cp.pose.orientation # Orientáció megtartása

        for i in range(steps):
            if not rclpy.ok(): return False

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = traj_x[i]
            msg.pose.position.y = traj_y[i]
            msg.pose.position.z = traj_z[i]

            self.servo_cp_pub.publish(msg)
            rate.sleep()

        return True

    def move_jaw_to(self, target, omega, dt):
        rate = self.create_rate(1.0/dt)

        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Várakozás a pofa adatokra...', throttle_duration_sec=2.0)
            rate.sleep()

        if not rclpy.ok(): return False

        if not self.measured_jaw.position: return False

        current_pos = self.measured_jaw.position[0]
        distance = abs(target - current_pos)

        if omega <= 0: return False
        T = distance / omega
        steps = int(T / dt)

        if steps == 0: return True

        traj = np.linspace(current_pos, target, steps)

        msg = JointState()
        msg.name = self.measured_jaw.name

        for i in range(steps):
            if not rclpy.ok(): return False

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [traj[i]]
            self.jaw_pub.publish(msg)
            rate.sleep()

        return True

def main(args=None):
    rclpy.init(args=args)
    server = GraspServer()

    # --- EZT A RÉSZT KOMMENTELD KI VAGY TÖRÖLD ---
    # A probléma forrása: blokkolja a futást a spin előtt,
    # így nem érkeznek meg a szenzoradatok.
    # server.move_tcp_to([0.0, 0.0, -0.12], 0.05, 0.01)
    # server.move_jaw_to(0.0, 0.1, 0.01)
    # server.get_logger().info('Robot resetelve...')
    # ---------------------------------------------

    executor = MultiThreadedExecutor()

    # A spin indítja el ténylegesen a kommunikációt
    try:
        rclpy.spin(server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
