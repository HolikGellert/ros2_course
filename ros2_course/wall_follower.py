import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # QoS beállítás a szimulátorhoz
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Paraméterek
        self.target_dist = 0.50  # Cél távolság a faltól (méter)

        # Zóna határok (mikor váltunk logikát)
        self.dist_too_close = 0.40
        self.dist_too_far = 0.60
        self.wall_lost_dist = 1.2  # Ha ennél messzebb a fal, akkor "elvesztettük" -> Külső sarok

        self.get_logger().info('Fejlett Wall Follower elindult (Sarkok kezelésével)!')

    def listener_callback(self, msg):
        # --- 1. LIDAR Adatok Szűrése ---
        # A "végtelen" értékeket lecseréljük 10 méterre
        ranges = [x if x != float('inf') else 10.0 for x in msg.ranges]

        # Zónák definiálása (TurtleBot3 specifikus indexek)
        # Elöl (0 fok +/- 20 fok)
        front_dist = min(ranges[0:20] + ranges[340:360])

        # Bal oldalon (két zónát nézünk a precizitásért)
        # Bal-Elöl (45 fok) - ez segít előre látni a görbületeket
        left_front_dist = min(ranges[30:60])
        # Bal (90 fok) - ez a pontos távolság a faltól
        left_dist = min(ranges[75:105])

        cmd = Twist()

        # --- 2. DÖNTÉSI FA (Prioritási sorrendben) ---

        # A) VÉSZHELYZET / BELSŐ SAROK (Konkáv)
        # Ha fal van előttünk, minden mást felülírunk és helyben fordulunk jobbra.
        if front_dist < 0.45:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8  # Gyors fordulás jobbra
            self.get_logger().info(f'BELSŐ SAROK! Fordulás jobbra. (Front: {front_dist:.2f})')

        # B) KÜLSŐ SAROK (Konvex) / FAL ELVESZTÉSE
        # Ha hirtelen eltűnt a fal balról (vagy a bal-első szenzor nem lát semmit)
        # Akkor élesen balra kanyarodunk, hogy "utána menjünk" a falnak.
        elif left_dist > self.wall_lost_dist or left_front_dist > self.wall_lost_dist:
            cmd.linear.x = 0.15   # Lassan előre
            cmd.angular.z = 0.6   # Éles fordulás balra (ívben)
            self.get_logger().info('KÜLSŐ SAROK! Ráfordulás balra...')

        # C) KORREKCIÓ: TÚL KÖZEL
        # Ha a falhoz túl közel sodródtunk, finoman jobbra tartunk.
        elif left_dist < self.dist_too_close:
            cmd.linear.x = 0.15
            cmd.angular.z = -0.3  # Finom jobbra
            self.get_logger().info(f'Korrekció: Túl közel ({left_dist:.2f})')

        # D) KORREKCIÓ: TÚL TÁVOL
        # Ha kicsit messzebb vagyunk a célnál (de még nem vesztettük el), finoman balra.
        elif left_dist > self.dist_too_far:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.3   # Finom balra
            self.get_logger().info(f'Korrekció: Túl távol ({left_dist:.2f})')

        # E) EGYENESEN
        # Ha a "folyosó" közepén vagyunk a tolerancián belül.
        else:
            cmd.linear.x = 0.25   # Gyorsabb haladás
            cmd.angular.z = 0.0
            self.get_logger().info('Egyenesen...')

        # --- 3. Parancs küldése ---
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
