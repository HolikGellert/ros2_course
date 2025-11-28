import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ros2_course_msgs.action import Grasp

class GraspClient(Node):

    def __init__(self):
        super().__init__('grasp_client')

        # 1. Action Client inicializálása
        self._action_client = ActionClient(self, Grasp, 'grasp')

        # 2. Feliratkozás a Marker-re
        self.subscription = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.marker_callback,
            10)

        self.goal_handle = None
        self.is_busy = False # Hogy ne küldjünk rá új parancsot, amíg az előző fut

        self.get_logger().info('Grasp Client elindult. Várakozás a markerre...')

    def marker_callback(self, msg):
        """Ez fut le, ha a dummy_marker publikál valamit."""
        if self.is_busy:
            # Opcionális: Ignorálhatjuk az új markert, ha épp dolgozunk
            return

        self.get_logger().info('Marker észlelve! Action indítása...')

        # Action Server elérhetőségének ellenőrzése
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('A Grasp szerver nem elérhető!')
            return

        # Cél összeállítása a marker pozíciójából
        goal_msg = Grasp.Goal()
        # A marker.pose.position egy geometry_msgs/Point, amit közvetlenül átadhatunk
        goal_msg.grasp_pos.x = msg.pose.position.x
        goal_msg.grasp_pos.y = msg.pose.position.y
        goal_msg.grasp_pos.z = msg.pose.position.z

        self.is_busy = True

        # Cél küldése (aszinkron)
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Ellenőrizzük, hogy a szerver elfogadta-e a kérést."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('A célt a szerver elutasította.')
            self.is_busy = False
            return

        self.get_logger().info('A célt a szerver elfogadta, folyamatban...')
        self.goal_handle = goal_handle

        # Várjuk az eredményt
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Visszajelzés a folyamatról (pl. "Nyitás...", "Mozgás...")"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.status}')

    def get_result_callback(self, future):
        """A végeredmény kezelése."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'SIKER! A művelet befejeződött.')
        else:
            self.get_logger().info(f'HIBA! A művelet sikertelen volt.')

        self.is_busy = False

def main(args=None):
    rclpy.init(args=args)
    action_client = GraspClient()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
