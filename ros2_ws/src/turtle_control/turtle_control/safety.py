import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class SafetyFilter(Node):
    def __init__(self):
        super().__init__('safety_filter')

        self.stop_distance = 0.20      # m, darunter wird blockiert
        self.range_timeout = 1.0       # s ohne Messwert -> blockieren

        self.last_range = None
        self.last_range_time = None
        self.blocked = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(Range, 'range', self.on_range, 10)

        self.get_logger().info('Safety filter aktiv')

    def on_range(self, msg):
        self.last_range = msg.range
        self.last_range_time = self.get_clock().now()

        was_blocked = self.blocked
        self.blocked = msg.range < self.stop_distance

        if self.blocked and not was_blocked:
            self.get_logger().warn(f'Hindernis bei {msg.range:.2f} m - gestoppt')
            self.pub.publish(Twist())
        elif was_blocked and not self.blocked:
            self.get_logger().info('Weg wieder frei')

    def sensor_stale(self):
        if self.last_range_time is None:
            return True
        age = (self.get_clock().now() - self.last_range_time).nanoseconds / 1e9
        return age > self.range_timeout

    def on_cmd(self, msg):
        if self.sensor_stale():
            self.pub.publish(Twist())
            return

        # Rueckwaerts und Drehen bleiben erlaubt, nur Vorwaerts wird blockiert
        if self.blocked and msg.linear.x > 0.0:
            out = Twist()
            out.angular.z = msg.angular.z
            self.pub.publish(out)
        else:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
