import math
import os
from typing import Optional, Tuple

import cv2
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    x = ax + math.cos(ath) * bx - math.sin(ath) * by
    y = ay + math.sin(ath) * bx + math.cos(ath) * by
    th = wrap_angle(ath + bth)
    return x, y, th


def pose_inverse(p):
    x, y, th = p
    c = math.cos(th)
    s = math.sin(th)
    return -c * x - s * y, s * x - c * y, -th


def pose_between(a_map, b_odom):
    return pose_compose(a_map, pose_inverse(b_odom))


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class RangeMapToOdomEstimator(Node):
    def __init__(self):
        super().__init__('range_map_to_odom_estimator')

        self.declare_parameter('map_yaml', '/home/deboshir/diplom/src/robot_pkg/maps/warehouse_map.yaml')
        self.declare_parameter('odom_topic', '/diff_drive/odometry')
        self.declare_parameter('front_topic', '/range_front')
        self.declare_parameter('left_topic', '/range_left')
        self.declare_parameter('right_topic', '/range_right')
        self.declare_parameter('transform_topic', '/map_to_odom_transform')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('front_sensor_pose', [0.21, 0.0, 0.0])
        self.declare_parameter('left_sensor_pose', [0.0, 0.21, math.pi / 2.0])
        self.declare_parameter('right_sensor_pose', [0.0, -0.21, -math.pi / 2.0])

        self.declare_parameter('max_sensor_range', 5.0)
        self.declare_parameter('ray_step', 0.01)

        self.declare_parameter('correction_period', 0.5)

        self.declare_parameter('optimizer_iterations', 5)
        self.declare_parameter('optimizer_step_xy', 0.1)
        self.declare_parameter('optimizer_step_yaw', 0.08)
        self.declare_parameter('numeric_eps_xy', 0.01)
        self.declare_parameter('numeric_eps_yaw', 0.02)

        self.declare_parameter('max_correction_xy', 0.08)
        self.declare_parameter('max_correction_yaw', 0.08)

        self.declare_parameter('smoothing_alpha_xy', 0.9)
        self.declare_parameter('smoothing_alpha_yaw', 0.9)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.front_sensor_pose = tuple(self.get_parameter('front_sensor_pose').value)
        self.left_sensor_pose = tuple(self.get_parameter('left_sensor_pose').value)
        self.right_sensor_pose = tuple(self.get_parameter('right_sensor_pose').value)

        self.max_sensor_range = float(self.get_parameter('max_sensor_range').value)
        self.ray_step = float(self.get_parameter('ray_step').value)

        self.optimizer_iterations = int(self.get_parameter('optimizer_iterations').value)
        self.optimizer_step_xy = float(self.get_parameter('optimizer_step_xy').value)
        self.optimizer_step_yaw = float(self.get_parameter('optimizer_step_yaw').value)
        self.numeric_eps_xy = float(self.get_parameter('numeric_eps_xy').value)
        self.numeric_eps_yaw = float(self.get_parameter('numeric_eps_yaw').value)

        self.max_correction_xy = float(self.get_parameter('max_correction_xy').value)
        self.max_correction_yaw = float(self.get_parameter('max_correction_yaw').value)

        self.smoothing_alpha_xy = float(self.get_parameter('smoothing_alpha_xy').value)
        self.smoothing_alpha_yaw = float(self.get_parameter('smoothing_alpha_yaw').value)

        self.map_image = None
        self.map_resolution = None
        self.map_origin = None
        self.occupied_thresh = None
        self.negate = 0

        self.load_map(self.get_parameter('map_yaml').value)

        self.odom_msg: Optional[Odometry] = None
        self.front_range: Optional[float] = None
        self.left_range: Optional[float] = None
        self.right_range: Optional[float] = None

        self.map_to_odom = (0.0, 0.0, 0.0)

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.get_parameter('front_topic').value, self.front_callback, 10)
        self.create_subscription(LaserScan, self.get_parameter('left_topic').value, self.left_callback, 10)
        self.create_subscription(LaserScan, self.get_parameter('right_topic').value, self.right_callback, 10)

        self.transform_pub = self.create_publisher(
            TransformStamped,
            self.get_parameter('transform_topic').value,
            10
        )

        self.create_timer(
            float(self.get_parameter('correction_period').value),
            self.correction_callback
        )

        self.get_logger().info('Range map->odom estimator started')

    def load_map(self, yaml_path: str):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        image_path = data['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f'Cannot load map image: {image_path}')

        self.map_image = image
        self.map_resolution = float(data['resolution'])
        self.map_origin = tuple(data['origin'])
        self.occupied_thresh = float(data.get('occupied_thresh', 0.65))
        self.negate = int(data.get('negate', 0))

        self.get_logger().info(
            f'Map loaded: {image_path}, size={image.shape[1]}x{image.shape[0]}, res={self.map_resolution}'
        )

    def extract_scan_range(self, msg: LaserScan) -> Optional[float]:
        finite = [r for r in msg.ranges if math.isfinite(r)]
        if not finite:
            return None
        value = min(finite)
        if value < msg.range_min or value > msg.range_max:
            return None
        return float(value)

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def front_callback(self, msg: LaserScan):
        self.front_range = self.extract_scan_range(msg)

    def left_callback(self, msg: LaserScan):
        self.left_range = self.extract_scan_range(msg)

    def right_callback(self, msg: LaserScan):
        self.right_range = self.extract_scan_range(msg)

    def odom_pose(self):
        if self.odom_msg is None:
            return None
        p = self.odom_msg.pose.pose.position
        q = self.odom_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return float(p.x), float(p.y), float(yaw)

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        ox, oy, _ = self.map_origin
        mx = int((x - ox) / self.map_resolution)
        my = int((y - oy) / self.map_resolution)
        img_y = self.map_image.shape[0] - 1 - my
        img_x = mx
        return img_x, img_y

    def is_occupied(self, x: float, y: float) -> bool:
        mx, my = self.world_to_map(x, y)

        if mx < 0 or my < 0 or mx >= self.map_image.shape[1] or my >= self.map_image.shape[0]:
            return True

        pixel = self.map_image[my, mx]
        occ = (255 - pixel) / 255.0 if self.negate == 0 else pixel / 255.0
        return occ >= self.occupied_thresh

    def sensor_world_pose(self, robot_pose, sensor_pose):
        return pose_compose(robot_pose, sensor_pose)

    def expected_range(self, robot_pose, sensor_pose) -> Optional[float]:
        sx, sy, sth = self.sensor_world_pose(robot_pose, sensor_pose)

        d = 0.0
        while d <= self.max_sensor_range:
            x = sx + d * math.cos(sth)
            y = sy + d * math.sin(sth)
            if self.is_occupied(x, y):
                return d
            d += self.ray_step

        return None

    def cost(self, pose):
        total = 0.0
        count = 0

        pairs = [
            (self.front_range, self.front_sensor_pose),
            (self.left_range, self.left_sensor_pose),
            (self.right_range, self.right_sensor_pose),
        ]

        for measured, sensor_pose in pairs:
            if measured is None:
                continue

            expected = self.expected_range(pose, sensor_pose)
            if expected is None:
                continue

            residual = measured - expected
            residual = max(min(residual, 2.0), -2.0)
            total += residual * residual
            count += 1

        if count == 0:
            return 1e9
        return total / count

    def optimize_pose(self, initial_pose):
        x, y, th = initial_pose

        for _ in range(self.optimizer_iterations):
            cx1 = self.cost((x + self.numeric_eps_xy, y, th))
            cx2 = self.cost((x - self.numeric_eps_xy, y, th))
            cy1 = self.cost((x, y + self.numeric_eps_xy, th))
            cy2 = self.cost((x, y - self.numeric_eps_xy, th))
            ct1 = self.cost((x, y, th + self.numeric_eps_yaw))
            ct2 = self.cost((x, y, th - self.numeric_eps_yaw))

            grad_x = (cx1 - cx2) / (2.0 * self.numeric_eps_xy)
            grad_y = (cy1 - cy2) / (2.0 * self.numeric_eps_xy)
            grad_th = (ct1 - ct2) / (2.0 * self.numeric_eps_yaw)

            x -= self.optimizer_step_xy * grad_x
            y -= self.optimizer_step_xy * grad_y
            th = wrap_angle(th - self.optimizer_step_yaw * grad_th)

        return x, y, th

    def apply_limited_smoothed_update(self, new_map_to_odom):
        old_x, old_y, old_th = self.map_to_odom
        new_x, new_y, new_th = new_map_to_odom

        dx = new_x - old_x
        dy = new_y - old_y
        dth = wrap_angle(new_th - old_th)

        dx = clamp(dx, -self.max_correction_xy, self.max_correction_xy)
        dy = clamp(dy, -self.max_correction_xy, self.max_correction_xy)
        dth = clamp(dth, -self.max_correction_yaw, self.max_correction_yaw)

        limited_x = old_x + dx
        limited_y = old_y + dy
        limited_th = wrap_angle(old_th + dth)

        alpha_xy = self.smoothing_alpha_xy
        alpha_yaw = self.smoothing_alpha_yaw

        smoothed_x = alpha_xy * old_x + (1.0 - alpha_xy) * limited_x
        smoothed_y = alpha_xy * old_y + (1.0 - alpha_xy) * limited_y

        yaw_delta = wrap_angle(limited_th - old_th)
        smoothed_th = wrap_angle(old_th + (1.0 - alpha_yaw) * yaw_delta)

        self.map_to_odom = (smoothed_x, smoothed_y, smoothed_th)

    def publish_map_to_odom_transform(self):
        if self.odom_msg is None:
            return

        tx, ty, tyaw = self.map_to_odom
        msg = TransformStamped()
        msg.header.stamp = self.odom_msg.header.stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame

        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(tyaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.transform_pub.publish(msg)

    def correction_callback(self):
        odom_pose = self.odom_pose()
        if odom_pose is None:
            return

        # если дальномеры ещё не готовы, всё равно публикуем текущее map->odom
        if self.front_range is None and self.left_range is None and self.right_range is None:
            self.publish_map_to_odom_transform()
            return

        estimated_map_pose = pose_compose(self.map_to_odom, odom_pose)
        corrected_map_pose = self.optimize_pose(estimated_map_pose)
        new_map_to_odom = pose_between(corrected_map_pose, odom_pose)
        self.apply_limited_smoothed_update(new_map_to_odom)
        self.publish_map_to_odom_transform()


def main(args=None):
    rclpy.init(args=args)
    node = RangeMapToOdomEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()