import math
from typing import List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Goals


class RouteSender(Node):
    def __init__(self) -> None:
        super().__init__('route_sender')

        self._client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses'
        )

        self.get_logger().info('Waiting for /navigate_through_poses action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server is available.')

        route = [
            ( 2.56, -1.84, 1.57),
            (0.54, -3.2627, 3.14),
            (-2.8, -1.88, -1.57),
            (0.1, 0.1, 1.57)
        ]

        self.send_route(route)

    def yaw_to_quaternion(self, yaw: float) -> Tuple[float, float]:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def send_route(self, route: List[Tuple[float, float, float]]) -> None:
        goal_msg = NavigateThroughPoses.Goal()

        goals_msg = Goals()
        goals_msg.goals = [self.make_pose(x, y, yaw) for x, y, yaw in route]

        goal_msg.poses = goals_msg
        goal_msg.behavior_tree = ''

        self.get_logger().info(f'Sending route with {len(route)} waypoints...')
        future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error('No response from action server.')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Route was rejected.')
            return

        self.get_logger().info('Route accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        # self.get_logger().info(
        #     f'Current waypoint index: {feedback.number_of_poses_remaining} poses remaining'
        # )

    def result_callback(self, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error('No result received.')
            rclpy.shutdown()
            return

        status = result.status
        self.get_logger().info(f'Route finished with status: {status}')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteSender()
    rclpy.spin(node)


if __name__ == '__main__':
    main()