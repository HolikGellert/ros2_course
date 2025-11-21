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
        self.wall_lost_dist = 1.2

        # --- PID Paraméterek ---
        # Csökkentettem a Kp-t, hogy ne legyen túl agresszív
        self.kp = 1.5

        # ### ÚJ ###: Kd (Derivative Gain) - A "lengéscsillapító"
        # Ez felel azért, hogy ne lendüljön túl a robot.
        self.kd = 10.0

        # ### ÚJ ###: Előző hiba tárolása a D-tag számításához
        self.prev_error = 0.0

        self.get_logger().info('Wall Follower elindult: PD-szabályozóval!')

    def listener_callback(self, msg):
        # 1. Szűrés
        ranges = [x if x != float('inf') else 10.0 for x in msg.ranges]

        # Irányok
        front_dist = min(ranges[0:20] + ranges[340:360])
        left_front_dist = min(ranges[30:60])
        left_dist = min(ranges[75:105])

        cmd = Twist()

        # --- A) VÉSZHELYZET (Front) ---
        if front_dist < 0.45:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8
            self.get_logger().info(f'AKADÁLY! (Front: {front_dist:.2f})')
            # Vészhelyzetben reseteljük a D-tagot, hogy ne zavarjon be később
            self.prev_error = 0.0

        # --- B) KÜLSŐ SAROK ---
        elif left_dist > self.wall_lost_dist or left_front_dist > self.wall_lost_dist:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.6
            self.get_logger().info('KÜLSŐ SAROK -> Ráfordulás balra...')
            self.prev_error = 0.0

        # --- C) NORMÁL ÜZEM: PD-SZABÁLYOZÁS ---
        else:
            # 1. Jelenlegi hiba számítása
            error = self.target_dist - left_dist

            # 2. ### ÚJ ###: D-tag számítása (Hiba változása)
            # Mennyit változott a hiba az előző kör óta?
            delta_error = error - self.prev_error

            # 3. A PD képlet: (P_erősítés * hiba) + (D_erősítés * változás)
            # Ha gyorsan közeledünk a falhoz, a delta_error ellentétes előjelű lesz, mint az error,
            # így fékezi a kormányzást.
            P_term = error * self.kp
            D_term = delta_error * self.kd

            angular_z = -1.0 * (P_term + D_term)

            # Limitálás
            cmd.angular.z = max(min(angular_z, 1.0), -1.0)

            # Dinamikus sebesség (kicsit óvatosabbra véve)
            if abs(error) < 0.1:
                cmd.linear.x = 0.30
            else:
                cmd.linear.x = 0.15

            # ### ÚJ ###: Jelenlegi hiba elmentése a következő körre
            self.prev_error = error

            self.get_logger().info(f'PD: Err={error:.2f}, Delta={delta_error:.3f}, Turn={cmd.angular.z:.2f}')

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
