#!/usr/bin/env python3

import time
import threading

from sub_behavior_interfaces.action import MoveDistance

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


# https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py

class MoveDistanceActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            MoveDistance,
            'move_distance',
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = MoveDistance.Feedback()
        feedback_msg.progress.x = 0.0

        for i in range(5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveDistance.Result()

            # Publish the feedback
            feedback_msg.progress.x = feedback_msg.progress.x + 0.1
            goal_handle.publish_feedback(feedback_msg)

            # delay for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = MoveDistance.Result()
        result.complete = True
        self.get_logger().info('Goal finished')
        return result


def main(args=None):
    rclpy.init(args=args)
    move_distance_server = MoveDistanceActionServer()
    rclpy.spin(move_distance_server, executor=MultiThreadedExecutor())
    move_distance_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
