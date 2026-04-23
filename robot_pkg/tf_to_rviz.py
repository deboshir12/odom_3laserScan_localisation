import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from tf2_ros import Buffer, TransformListener, LookupException

from typing import Optional


class TumTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('tum_trajectory_recorder')

        # параметры
        self.declare_parameter('pose_array_topic', '/diff_drive/pose')
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        pose_topic = self.get_parameter('pose_array_topic').value
        self.base_name = self.get_parameter('base_link_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # файлы
        self.gt_file = open('gt.txt', 'w')
        self.est_file = open('est.txt', 'w')

        # подписка на ground truth
        self.create_subscription(
            PoseArray,
            pose_topic,
            self.gt_callback,
            10
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # таймер для оценки
        self.create_timer(0.05, self.record_estimate)

        self.get_logger().info('TUM trajectory recorder started')

    # -------------------------------
    # Ground truth из PoseArray
    # -------------------------------
    def gt_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        # ВАЖНО:
        # PoseArray не содержит имён, поэтому предполагаем:
        # base_link = первый элемент
        pose = msg.poses[2]
        # self.get_logger().info(msg.poses)

        # время
        now = self.get_clock().now().to_msg()
        t = now.sec + now.nanosec * 1e-9

        p = pose.position
        q = pose.orientation

        line = f"{t} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n"
        self.gt_file.write(line)

    # -------------------------------
    # Оценка из TF (map -> base_link)
    # -------------------------------
    def record_estimate(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except LookupException:
            return
        
        now = self.get_clock().now().to_msg()
        t = now.sec + now.nanosec * 1e-9

        p = tf.transform.translation
        q = tf.transform.rotation

        line = f"{t} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n"
        self.est_file.write(line)

    def destroy_node(self):
        self.gt_file.close()
        self.est_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TumTrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()