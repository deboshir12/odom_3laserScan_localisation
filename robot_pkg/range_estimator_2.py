# from typing import Optional

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster


# class MapOdomTfBroadcaster(Node):
#     def __init__(self):
#         super().__init__('map_odom_tf_broadcaster')

#         self.declare_parameter('odom_topic', '/diff_drive/odometry')
#         self.declare_parameter('transform_topic', '/map_to_odom_transform')
#         self.declare_parameter('tf_publish_period', 0.01)

#         self.latest_odom: Optional[Odometry] = None
#         self.latest_transform: Optional[TransformStamped] = None

#         self.create_subscription(
#             Odometry,
#             self.get_parameter('odom_topic').value,
#             self.odom_callback,
#             10
#         )
#         self.create_subscription(
#             TransformStamped,
#             self.get_parameter('transform_topic').value,
#             self.transform_callback,
#             10
#         )

#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.create_timer(
#             float(self.get_parameter('tf_publish_period').value),
#             self.publish_tf_callback
#         )

#         self.get_logger().info('Map->odom TF broadcaster started')

#     def odom_callback(self, msg: Odometry):
#         self.latest_odom = msg

#     def transform_callback(self, msg: TransformStamped):
#         self.latest_transform = msg

#     def publish_tf_callback(self):
#         if self.latest_transform is None or self.latest_odom is None:
#             return

#         msg = TransformStamped()
#         msg.header.stamp = self.latest_odom.header.stamp
#         msg.header.frame_id = self.latest_transform.header.frame_id
#         msg.child_frame_id = self.latest_transform.child_frame_id
#         msg.transform = self.latest_transform.transform

#         self.tf_broadcaster.sendTransform(msg)



# def main(args=None):
#     rclpy.init(args=args)
#     node = MapOdomTfBroadcaster()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()








from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MapOdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('map_odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/diff_drive/odometry')
        self.declare_parameter('tf_publish_period', 0.01)

        self.latest_odom: Optional[Odometry] = None

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(
            float(self.get_parameter('tf_publish_period').value),
            self.publish_tf_callback
        )

        self.get_logger().info('Publishing identity transform map -> odom')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def publish_tf_callback(self):
        if self.latest_odom is None:
            return

        msg = TransformStamped()
        msg.header.stamp = self.latest_odom.header.stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"

        # identity transform
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0

        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()