import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameter der Fahrfigur
        self.linear_speed = 0.5      # m/s
        self.angular_speed = 0.5     # rad/s
        self.side_time = 4.0         # s geradeaus
        self.turn_time = 3.14        # s drehen (90 Grad bei 0.5 rad/s)

        self.phase = 0
        self.sides_done = 0
        self.phase_start = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('Square driver gestartet')

    def elapsed(self):
        return (self.get_clock().now() - self.phase_start).nanoseconds / 1e9

    def tick(self):
        msg = Twist()

        if self.phase == 0:          # geradeaus
            msg.linear.x = self.linear_speed
            if self.elapsed() >= self.side_time:
                self.next_phase(1)

        elif self.phase == 1:        # drehen
            msg.angular.z = self.angular_speed
            if self.elapsed() >= self.turn_time:
                self.sides_done += 1
                if self.sides_done >= 4:
                    self.next_phase(2)
                else:
                    self.next_phase(0)

        else:                        # fertig
            self.pub.publish(Twist())
            self.get_logger().info('Viereck abgefahren')
            self.timer.cancel()
            return

        self.pub.publish(msg)

    def next_phase(self, phase):
        self.pub.publish(Twist())
        self.phase = phase
        self.phase_start = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
