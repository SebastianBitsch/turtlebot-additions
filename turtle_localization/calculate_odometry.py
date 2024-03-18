import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class CalculateOdometry(Node):

    def __init__(self):
        super().__init__('calculate_odometry')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.ranges)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CalculateOdometry()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()