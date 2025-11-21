import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Konfigurációs Paraméterek ---
        self.target_dist = 0.50

        # Két küszöbértékünk van:
        self.wall_lost_dist = 1.2  # Ha ennél nagyobb: Külső sarok (kanyarodni kell)
        self.max_wall_dist = 2.0   # Ha ennél is nagyobb: Nincs fal (egyenesen kell menni)

        # --- PID Paraméterek ---
        self.kp = 1.5
        self.kd = 10.0
        self.prev_error = 0.0

        self.get_logger().info('Wall Follower elindult: Keresés funkcióval bővítve!')

    def listener_callback(self, msg):
        # 1. Szűrés
        ranges = [x if x != float('inf') else 10.0 for x in msg.ranges]

        # Irányok
        front_dist = min(ranges[0:20] + ranges[340:360])
        left_front_dist = min(ranges[30:60])
        left_dist = min(ranges[75:105])

        cmd = Twist()

        # --- A) VÉSZHELYZET (Front) ---
        # Ez a legfontosabb: Ha bármi van elöl, megállunk/fordulunk.
        if front_dist < 0.45:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8
            self.get_logger().info(f'AKADÁLY! (Front: {front_dist:.2f}) -> Fordulás jobbra')
            self.prev_error = 0.0

        # --- B1) NINCS FAL / KERESÉS (ÚJ RÉSZ) ---
        # Ha a bal oldali fal nagyon messze van (> 2 méter), ne körözzünk!
        # Menjünk egyenesen, amíg (A) miatt falnak nem megyünk, vagy (C) miatt meg nem találjuk a falat.
        elif left_dist > self.max_wall_dist:
            cmd.linear.x = 0.30   # Gyorsabb keresés
            cmd.angular.z = 0.0   # Csak egyenesen!
            self.get_logger().info('Nincs fal -> Keresés egyenesen...')
            self.prev_error = 0.0

        # --- B2) KÜLSŐ SAROK ---
        # A fal messzebb van mint 1.2, DE közelebb mint 2.0.
        # Tehát valószínűleg épp most fogyott el -> Kanyarodjunk utána.
        elif left_dist > self.wall_lost_dist:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.6
            self.get_logger().info('KÜLSŐ SAROK -> Ráfordulás balra...')
            self.prev_error = 0.0

        # --- C) NORMÁL ÜZEM: PD-SZABÁLYOZÁS ---
        else:
            # Itt a left_dist <= 1.2, tehát látjuk a falat, lehet szabályozni.
            error = self.target_dist - left_dist
            delta_error = error - self.prev_error

            P_term = error * self.kp
            D_term = delta_error * self.kd

            angular_z = -1.0 * (P_term + D_term)
            cmd.angular.z = max(min(angular_z, 1.0), -1.0)

            # Dinamikus sebesség
            if abs(error) < 0.1:
                cmd.linear.x = 0.30
            else:
                cmd.linear.x = 0.15

            self.prev_error = error
            self.get_logger().info(f'PD: Err={error:.2f}, Turn={cmd.angular.z:.2f}')

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
