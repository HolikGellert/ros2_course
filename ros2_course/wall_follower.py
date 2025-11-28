import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # QoS Profile: Required for Gazebo compatibility (Best Effort)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Configuration Parameters ---
        self.target_dist = 0.50

        # State thresholds
        self.wall_lost_dist = 1.2  # > 1.2m: Outer corner detected (Turn Left)
        self.max_wall_dist = 2.0   # > 2.0m: No wall detected (Search Mode)

        # --- PD Controller Gains ---
        self.kp = 1.5   # Proportional gain
        self.kd = 10.0  # Derivative gain (damping)
        self.prev_error = 0.0

        self.get_logger().info('Wall Follower started with PD & Search Logic!')

    def listener_callback(self, msg):
        # 1. Data Filtering: Replace 'inf' with 10.0 to prevent math errors
        ranges = [x if x != float('inf') else 10.0 for x in msg.ranges]

        # 2. Define Sensor Zones (TurtleBot3 specific indices)
        front_dist = min(ranges[0:20] + ranges[340:360])
        left_front_dist = min(ranges[30:60])
        left_dist = min(ranges[75:105])

        cmd = Twist()

        # --- A) EMERGENCY: Obstacle ahead ---
        # Priority 1: Avoid collision
        if front_dist < 0.45:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8  # Spot turn Right
            self.get_logger().info(f'OBSTACLE! (Front: {front_dist:.2f}) -> Turning Right')
            self.prev_error = 0.0

        # --- B1) SEARCH MODE: No wall detected ---
        # Wall is very far (> 2.0m). Drive straight to find one.
        elif left_dist > self.max_wall_dist:
            cmd.linear.x = 0.30   # Fast forward
            cmd.angular.z = 0.0
            self.get_logger().info('No wall -> Searching straight...')
            self.prev_error = 0.0

        # --- B2) OUTER CORNER: Wall ended recently ---
        # Wall is between 1.2m and 2.0m. Turn left to wrap around the corner.
        elif left_dist > self.wall_lost_dist:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.6
            self.get_logger().info('OUTER CORNER -> Turning Left...')
            self.prev_error = 0.0

        # --- C) WALL FOLLOWING: PD Control ---
        # Wall is within range (< 1.2m). Maintain target distance.
        else:
            # Calculate Error
            error = self.target_dist - left_dist
            delta_error = error - self.prev_error

            # PD Formula
            P_term = error * self.kp
            D_term = delta_error * self.kd

            # Calculate steering (Negative because positive error means we are too close)
            angular_z = -1.0 * (P_term + D_term)
            cmd.angular.z = max(min(angular_z, 1.0), -1.0) # Limit speed

            # Dynamic Speed: Fast on straights, slow on corrections
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
