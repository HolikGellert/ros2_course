import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import Marker

from ros2_course_msgs.action import Grasp


class GraspClient(Node):

    def __init__(self):
        super().__init__('grasp_client')

        # 1. Action Kliens Létrehozása
        # Csatlakozik a 'grasp' nevű action szerverhez
        self._action_client = ActionClient(self, Grasp, 'grasp')

        # 2. Marker Feliratkozás Létrehozása
        # Figyeli a dummy markert
        self.subscription_marker = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.marker_callback, # A callback, ami elindítja az actiont
            10)

        # Zászló, ami megakadályozza, hogy egyszerre több célt küldjünk,
        # ha a marker gyorsan frissülne.
        self._is_goal_active = False

        self.get_logger().info('Grasp kliens elindult, várakozik a /dummy_target_marker topicra...')

    def marker_callback(self, msg):
        """
        Ez a függvény hívódik meg, amikor egy új Marker üzenet érkezik.
        """
        # Ha már folyamatban van egy megfogás, ne küldjünk újat
        if self._is_goal_active:
            self.get_logger().warn('Már egy aktív fogási folyamat zajlik, az új marker figyelmen kívül hagyva.')
            return

        # Zászló beállítása
        self._is_goal_active = True
        self.get_logger().info('Marker észlelve! Várakozás az action szerverre (/grasp)...')

        # 3. Várakozás a Szerverre
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action szerver nem található! A cél küldése megszakítva.')
            self._is_goal_active = False # Zászló visszaállítása
            return

        self.get_logger().info('Action szerver megtalálva. Cél (Goal) küldése...')

        # 4. Cél (Goal) Definiálása
        goal_msg = Grasp.Goal()

        # A Grasp.action definíció (geometry_msgs/Point grasp_pos)
        # és a Marker (msg.pose.position, ami szintén Point)
        # közvetlenül kompatibilisek.
        goal_msg.grasp_pos = msg.pose.position

        # 5. Cél Elküldése (Aszinkron)
        # Elküldjük a célt, és megadjuk a feedback callback-et
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Hozzáadunk egy callback-et, ami akkor fut le,
        # amikor a szerver válaszol (elfogadta/elutasította a célt)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Visszajelzés (Feedback) fogadása a szervertől futás közben."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback érkezett: {feedback.status}')

    def goal_response_callback(self, future):
        """
        Akkor hívódik meg, amikor a szerver elfogadja vagy elutasítja a célt.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('A célt a szerver elutasította.')
            self._is_goal_active = False # Zászló visszaállítása
            return

        self.get_logger().info('Cél elfogadva a szerver által. Várakozás az eredményre...')

        # Ha elfogadta, kérjük az eredményt (aszinkron módon)
        self._get_result_future = goal_handle.get_result_async()
        # Hozzáadunk egy callback-et, ami az action befejeződésekor hívódik meg
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Akkor hívódik meg, amikor az action befejeződött (Result)."""
        result = future.result().result

        # Kiírjuk az eredményt
        self.get_logger().info(f'--- Eredmény')
        self.get_logger().info(f'Sikeres: {result.success}')
        self.get_logger().info(f'-----------------')

        # A folyamat befejeződött, készen állunk egy új marker fogadására
        self._is_goal_active = False


def main(args=None):
    rclpy.init(args=args)

    grasp_client = GraspClient()

    try:
        # A spin() futtatja a node-ot, és engedi, hogy a callback-ek (pl. marker_callback)
        # meghívódjanak, amikor üzenetek érkeznek.
        rclpy.spin(grasp_client)
    except KeyboardInterrupt:
        pass
    finally:
        # Tiszta leállítás
        grasp_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
