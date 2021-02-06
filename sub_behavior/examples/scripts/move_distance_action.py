import time

from sub_behavior_interfaces.action import MoveDistance

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class MoveDistanceActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')

        self._action_server = ActionServer(
            self,
            MoveDistance,
            'move_distance',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        print(goal_request)
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

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

        print(goal_handle.request.distances)
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
        return result


def main(args=None):
    rclpy.init(args=args)
    move_distance_server = MoveDistanceActionServer()
    rclpy.spin(move_distance_server)
    move_distance_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
