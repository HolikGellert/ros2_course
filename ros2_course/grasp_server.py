import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ros2_course_msgs.action import Grasp

class GraspServer(Node):

    def __init__(self):
        super().__init__('grasp_server')

        # Állapot-feliratkozások
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

        # Parancs-publisherek
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

        self.get_logger().info('Grasp action server sikeresen elindult, vár a célokra...')

    # Robot Callback-ek

    def cb_measured_cp(self, msg):
        """Callback a robot mért TCP pozíciójára."""
        self.measured_cp = msg

    def cb_measured_jaw(self, msg):
        """Callback a robot mért pofa-állására."""
        self.measured_jaw = msg

    # Action Server Callback-ek

    def goal_callback(self, goal_request):
        self.get_logger().info('Új cél érkezett, elfogadva.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cél megszakítása kérés érkezett, elfogadva.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Cél végrehajtása...')

        # Feedback és Result üzenetek
        feedback_msg = Grasp.Feedback()
        result = Grasp.Result()

        # Mozgatási paraméterek
        v = 0.01
        omega = 0.1
        dt = 0.01

        # 1. LÉPÉS: Gripper nyitása
        feedback_msg.status = 'Gripper nyitása...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        # Végrehajtjuk a mozgást
        success = self.move_jaw_to(target=0.8, omega=omega, dt=dt)
        if not success:
            goal_handle.abort() # Ha a mozgás sikertelen (pl. node leáll), megszakítjuk
            result.success = False
            result.message = 'Hiba a pofa mozgatása közben (nyitás).'
            return result

        # Ellenőrizzük, hogy törölték-e a célt
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Cél megszakítva a gripper nyitása közben.'
            return result

        # 2. LÉPÉS: Mozgás a célpozícióba
        feedback_msg.status = 'Mozgás a célpozícióba...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        # Célkoordináták kiolvasása a goal-ból
        target_pose = goal_handle.request.target_pose
        target_cam = np.array([target_pose.position.x,
                               target_pose.position.y,
                               target_pose.position.z])

        # Transzformáció (az eredeti psm_grasp.py-ból)
        t_base_cam = np.array([0.18, 0.03, 0.01])
        R_base_cam = np.array([[-0.57922797, -0.27504447, -0.76736269],
                               [-0.40557979,  0.9138093,  -0.02139151],
                               [ 0.70710678,  0.29883624, -0.64085638]])

        # Célpont a bázis koordináta-rendszerben + pre-grasp offset
        target_base = (R_base_cam @ target_cam) + t_base_cam
        target_base = target_base + np.array([0.0, 0.0, 0.008]) # 8mm-rel a marker fölé

        # Mozgás a kiszámított célponthoz
        success = self.move_tcp_to(target=target_base, v=v, dt=dt)
        if not success:
            goal_handle.abort()
            result.success = False
            result.message = 'Hiba a TCP mozgatása közben.'
            return result

        # Ellenőrizzük, hogy törölték-e a célt
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Cél megszakítva a pozícióra mozgás közben.'
            return result

        # 3. LÉPÉS: Gripper zárása
        feedback_msg.status = 'Gripper zárása...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        success = self.move_jaw_to(target=0.0, omega=omega, dt=dt)
        if not success:
            goal_handle.abort()
            result.success = False
            result.message = 'Hiba a pofa mozgatása közben (zárás).'
            return result

        # 4. LÉPÉS: Befejezés
        goal_handle.succeed()
        result.success = True
        result.message = 'Megfogás sikeresen végrehajtva!'
        self.get_logger().info(result.message)
        return result

    # Mozgató segédfüggvények (a psm_grasp.py-ból átemelve és javítva)
    # FONTOS: Ezek a függvények most már nem hívják a rclpy.spin_once()-t.
    # Helyette a 'main' függvényben MultiThreadedExecutor-t használunk,
    # így a háttérben futó subscriber callback-ek frissíteni tudják
    # a self.measured_cp és self.measured_jaw változókat, miközben
    # ezek a (blokkoló) move függvények futnak.

    def move_tcp_to(self, target, v, dt):
        """A robot TCP-jét a megadott célpontba mozgatja."""
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Várakozás, amíg megkapjuk az első pozíció adatot
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Várakozás a /PSM1/measured_cp-re...', throttle_duration_sec=1.0)
            loop_rate.sleep() # Hagyjuk, hogy a többi thread fusson

        if not rclpy.ok():
            return False # Node leállt

        # Mozgás előkészítése
        loop_rate = self.create_rate(1.0/dt, self.get_clock())
        pos_curr_np = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z])
        pos_target_np = np.array(target)
        distance = np.linalg.norm(pos_target_np - pos_curr_np)

        if v == 0:
            self.get_logger().error("A TCP sebesség (v) nulla!")
            return False

        T = distance / v
        N = int(round(abs(T / dt)))

        if N == 0:
            return True # Már ott vagyunk

        tr_x = np.linspace(start=self.measured_cp.pose.position.x, stop=target[0], num=N)
        tr_y = np.linspace(start=self.measured_cp.pose.position.y, stop=target[1], num=N)
        tr_z = np.linspace(start=self.measured_cp.pose.position.z, stop=target[2], num=N)

        # Sablon üzenet (orientáció és frame_id megtartásához)
        # FONTOS: Soha ne módosítsuk a self.measured_cp-t!
        msg_template = self.measured_cp

        for i in range(N):
            if not rclpy.ok():
                return False # Node leállt

            # Új üzenet létrehozása minden lépésben
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = msg_template.header.frame_id
            msg.pose.position.x = tr_x[i]
            msg.pose.position.y = tr_y[i]
            msg.pose.position.z = tr_z[i]
            msg.pose.orientation = msg_template.pose.orientation

            self.servo_cp_pub.publish(msg)
            loop_rate.sleep() # Várakozás a következő ciklusig

        return True # Sikeres mozgás

    def move_jaw_to(self, target, omega, dt):
        """A robot pofáit a megadott cél-nyitottságra mozgatja."""
        loop_rate = self.create_rate(100, self.get_clock())

        # Várakozás az első pofa-pozícióra
        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Várakozás a /PSM1/jaw/measured_js-re...', throttle_duration_sec=1.0)
            loop_rate.sleep()

        if not rclpy.ok():
            return False # Node leállt

        loop_rate = self.create_rate(1.0/dt, self.get_clock())

        # Ellenőrizzük, hogy van-e pozíció az üzenetben
        if not self.measured_jaw.position:
            self.get_logger().error("A mért pofa üzenet nem tartalmaz pozíciót!")
            return False

        current_pos = self.measured_jaw.position[0]
        distance = current_pos - target

        if omega == 0:
            self.get_logger().error("A pofa sebessége (omega) nulla!")
            return False

        T = abs(distance / omega)
        N = int(round(abs(T / dt)))

        if N == 0:
            return True # Már ott vagyunk

        tr_jaw = np.linspace(start = current_pos, stop = target, num = N)

        # Sablon üzenet
        msg_template = self.measured_jaw

        for i in range(N):
            if not rclpy.ok():
                return False # Node leállt

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = msg_template.name
            msg.velocity = [] # A velocity/effort nem szükséges a parancshoz
            msg.effort = []

            msg.position = [tr_jaw[i]]

            self.jaw_pub.publish(msg)
            loop_rate.sleep()

        return True # Sikeres mozgás


def main(args=None):
    rclpy.init(args=args)

    grasp_server = GraspServer()

    grasp_server.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    grasp_server.move_jaw_to(0.0, 0.1, 0.01)
    grasp_server.get_logger().info('Robot resetelve. A szerver készen áll.')

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(grasp_server, executor=executor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        grasp_server.get_logger().error(f"Hiba történt a spin közben: {e}")
    finally:
        grasp_server.get_logger().info('Action server leállítása...')
        grasp_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
