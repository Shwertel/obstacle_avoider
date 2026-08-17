import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')

        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.7)  # must be this far before resuming forward
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('cone_angle_deg', 30.0)

        self.enabled = False
        self.avoiding = False        # are we currently in "turning to avoid" mode?
        self.turn_direction = 1.0    # +1 = left, -1 = right; locked in once avoidance starts

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.start_service = self.create_service(Trigger, 'start_avoidance', self.start_callback)
        self.stop_service = self.create_service(Trigger, 'stop_avoidance', self.stop_callback)

        self.get_logger().info('Obstacle avoider node ready. Call /start_avoidance to begin.')

    def start_callback(self, request, response):
        self.enabled = True
        self.avoiding = False
        response.success = True
        response.message = 'Started'
        return response

    def stop_callback(self, request, response):
        self.enabled = False
        self.publisher.publish(Twist())
        response.success = True
        response.message = 'Stopped'
        return response

    def scan_callback(self, msg):
        if not self.enabled:
            return

        safe_distance = self.get_parameter('safe_distance').value
        clear_distance = self.get_parameter('clear_distance').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        cone_angle_deg = self.get_parameter('cone_angle_deg').value

        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        cone_size = int((cone_angle_deg / 360.0) * num_readings)
        left_ranges = [r for r in msg.ranges[:cone_size] if r > 0.0]   # roughly front-left
        right_ranges = [r for r in msg.ranges[-cone_size:] if r > 0.0]  # roughly front-right
        all_valid = left_ranges + right_ranges

        if not all_valid:
            return

        closest = min(all_valid)
        cmd = Twist()

        if not self.avoiding:
            # Not currently avoiding — check if something just got too close
            if closest < safe_distance:
                self.avoiding = True
                # Pick the direction with MORE open space, turn away from the obstacle
                left_min = min(left_ranges) if left_ranges else float('inf')
                right_min = min(right_ranges) if right_ranges else float('inf')
                self.turn_direction = 1.0 if right_min < left_min else -1.0
                self.get_logger().info(
                    f'Obstacle at {closest:.2f} m — starting avoidance, '
                    f'turning {"left" if self.turn_direction > 0 else "right"}'
                )

        if self.avoiding:
            # Stay in turn mode until CLEARLY clear (hysteresis via clear_distance > safe_distance)
            if closest > clear_distance:
                self.avoiding = False
                self.get_logger().info(f'Path clear ({closest:.2f} m) — resuming forward')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                self.publisher.publish(cmd)
                return

        # Not avoiding: drive forward
        cmd.linear.x = linear_speed
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
