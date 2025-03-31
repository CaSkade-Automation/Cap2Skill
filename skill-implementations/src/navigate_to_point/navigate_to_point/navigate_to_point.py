import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigateToPointSkill(Node):
    def __init__(self):
        super().__init__("navigate_to_point")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: Remaining distance: {0}'.format(feedback.distance_remaining))

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        if result: 
            self.get_logger().info('Goal reached successfully!')
        else: 
            self.get_logger().info('Failed to reach goal')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    navigate_to_point_skill = NavigateToPointSkill()

    future = navigate_to_point_skill.send_goal(-4.21, -3.02)
    rclpy.spin(navigate_to_point_skill, future)

if __name__ == '__main__':
    main()