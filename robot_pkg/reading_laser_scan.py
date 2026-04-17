import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RangePrinterNode(Node):
    def __init__(self):
        super().__init__('range_printer_node')

        self.distances = {
            'front': None,
            'left': None,
            'right': None,
        }

        self.create_subscription(
            LaserScan,
            '/range_front_raw',
            self.front_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            '/range_left_raw',
            self.left_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            '/range_right_raw',
            self.right_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.print_distances)

    def extract_distance(self, msg: LaserScan):
        if not msg.ranges:
            return None

        value = msg.ranges[0]

        if math.isinf(value) or math.isnan(value):
            return None

        return value

    def front_callback(self, msg: LaserScan):
        self.distances['front'] = self.extract_distance(msg)

    def left_callback(self, msg: LaserScan):
        self.distances['left'] = self.extract_distance(msg)

    def right_callback(self, msg: LaserScan):
        self.distances['right'] = self.extract_distance(msg)

    def format_value(self, value):
        if value is None:
            return 'нет данных'
        return f'{value:.3f} м'

    def print_distances(self):
        front = self.format_value(self.distances['front'])
        left = self.format_value(self.distances['left'])
        right = self.format_value(self.distances['right'])

        self.get_logger().info(
            f'Front: {front} | Left: {left} | Right: {right}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RangePrinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()