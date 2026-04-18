import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MultiScanFixer(Node):
    def __init__(self):
        super().__init__('multi_scan_fixer')

        self.sensors = {
            'lidar': {
                'input_topic': '/range_lidar_raw',
                'output_topic': '/scan',
                'frame_id': 'lidar_sensor',
            },
            'front': {
                'input_topic': '/range_front_raw',
                'output_topic': '/range_front',
                'frame_id': 'range_front',
            },
            'left': {
                'input_topic': '/range_left_raw',
                'output_topic': '/range_left',
                'frame_id': 'range_left',
            },
            'right': {
                'input_topic': '/range_right_raw',
                'output_topic': '/range_right',
                'frame_id': 'range_right',
            },
        }

        self.pubs = {}

        for name, cfg in self.sensors.items():
            self.pubs[name] = self.create_publisher(
                LaserScan,
                cfg['output_topic'],
                10
            )

            self.create_subscription(
                LaserScan,
                cfg['input_topic'],
                self.make_callback(name),
                10
            )

        self.get_logger().info('Multi scan fixer started')

    def make_callback(self, sensor_name):
        def callback(msg: LaserScan):
            fixed_msg = LaserScan()

            fixed_msg.header = msg.header
            fixed_msg.header.frame_id = self.sensors[sensor_name]['frame_id']

            fixed_msg.angle_min = msg.angle_min
            fixed_msg.angle_max = msg.angle_max
            fixed_msg.angle_increment = msg.angle_increment
            fixed_msg.time_increment = msg.time_increment
            fixed_msg.scan_time = msg.scan_time
            fixed_msg.range_min = msg.range_min
            fixed_msg.range_max = msg.range_max
            fixed_msg.ranges = list(msg.ranges)
            fixed_msg.intensities = list(msg.intensities)

            self.pubs[sensor_name].publish(fixed_msg)

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = MultiScanFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()