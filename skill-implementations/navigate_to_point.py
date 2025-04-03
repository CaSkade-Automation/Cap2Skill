import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/navigateToPoint/navigateToPoint", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/navigateToPoint/navigateToPoint", description = "Navigate mobile robot to a desired goal point.")
class NavigateToPointSkill(ROS2Skill):
    def __init__(self):
        super().__init__("navigate_to_point")

        self.position = Point()
        self._action_client = None
        self.goal_handle = None
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False

    @skill_parameter(is_required=True, name="position_x", description="The x coordinate of the goal position.")
    def get_position_x(self) -> float:
        return self.position.x
    
    @skill_parameter(is_required=True, name="position_y", description="The y coordinate of the goal position.")
    def get_position_y(self) -> float:
        return self.position.y
    
    @starting
    def starting(self) -> None:
        self.node.get_logger().info("NavigateToPointSkill is starting with creating an action client to navigate_to_pose action server")
        self._action_client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    @execute
    def execute(self) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.position.x
        goal_msg.pose.pose.position.y = self.position.y
        goal_msg.pose.pose.position.z = 0.0
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        while not self.goal_reached or not self.goal_rejected or not self.goal_failed:
            self.node.get_logger().info("Waiting for goal to be reached...")
            time.sleep(1)
        
        if self.goal_rejected:
            self.node.get_logger().info("Goal was rejected.")
            self.state_machine.abort()
        elif self.goal_failed:
            self.node.get_logger().info("Goal failed.")
            self.state_machine.abort()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info('Feedback: Remaining distance: {0}'.format(feedback.distance_remaining))

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            self.goal_rejected = True
            return
        
        self.node.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        if result: 
            self.node.get_logger().info('Goal reached successfully!')
            self.goal_reached = True
        else: 
            self.node.get_logger().info('Failed to reach goal')
            self.goal_failed = True

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Goal successfully canceled')
            self.cancel_success = True
        else:
            self.node.get_logger().info('Goal failed to cancel')
            self.cancel_failed = True

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"NavigateToPointSkill is completing with position: x={self.position.x}, y={self.position.y}")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("NavigateToPointSkill is stopping with canceling the goal")
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        while not self.cancel_success or not self.cancel_failed:
            self.node.get_logger().info("Waiting for goal to be canceled...")
            time.sleep(1)
        if self.cancel_failed:
            self.node.get_logger().info("Goal failed to cancel.")
            self.state_machine.abort()

    @aborting
    def aborting(self) -> None:
        self.node.get_logger().info("NavigateToPointSkill is aborting")
        velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0.0
        velocity_cmd.linear.y = 0.0
        velocity_cmd.linear.z = 0.0
        velocity_cmd.angular.x = 0.0
        velocity_cmd.angular.y = 0.0
        velocity_cmd.angular.z = 0.0
        velocity_publisher.publish(velocity_cmd)
        velocity_publisher.destroy()

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("NavigateToPointSkill is resetting")
        self._action_client.destroy()
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False