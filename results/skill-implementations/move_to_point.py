import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, Point, Quaternion

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/moveToPoint/moveToPoint", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/moveToPoint/moveToPoint", 
       description="Move the manipulator of the mobile robot to a desired goal point.")
class MoveToPointSkill(ROS2Skill):
    def __init__(self):
        super().__init__("move_to_point")

        self.goal_position = Point()
        self.robot_position = Point()
        self._action_client = None
        self.goal_handle = None
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False

    @skill_parameter(is_required=True, name="goal_position_x", description="The x coordinate of the goal position.")
    def get_goal_position_x(self) -> float:
        return self.goal_position.x
    
    @skill_parameter(is_required=True, name="goal_position_y", description="The y coordinate of the goal position.")
    def get_goal_position_y(self) -> float:
        return self.goal_position.y
    
    @skill_parameter(is_required=True, name="goal_position_z", description="The z coordinate of the goal position.")
    def get_goal_position_z(self) -> float:
        return self.goal_position.z
    
    @skill_output(is_required=True, name="robot_position_x", description="Indicates if the x coordinate of the goal was reached successfully.")
    def get_robot_position_x(self) -> float:
        return self.robot_position.x
    
    @skill_output(is_required=True, name="robot_position_y", description="Indicates if the y coordinate of the goal was reached successfully.")
    def get_robot_position_y(self) -> float:
        return self.robot_position.y
    
    @skill_output(is_required=True, name="robot_position_z", description="Indicates if the z coordinate of the goal was reached successfully.")
    def get_robot_position_z(self) -> float:
        return self.robot_position.z
    
    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MoveToPointSkill is starting with creating an action client to move_action action server")
        self._action_client = ActionClient(self.node, MoveGroup, "move_action")

    @execute
    def execute(self) -> None:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.start_state.joint_state.header.frame_id = "base_link"
        goal_msg.request.goal_constraints.append(self.create_goal_constraint())
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

    def create_goal_constraint(self):
        # Create a goal constraint for the manipulator
        pose = Pose()
        pose.position = self.goal_position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # Add constraints here as needed
        return pose

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
            self.robot_position = self.goal_position
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
        self.node.get_logger().info(f"MoveToPointSkill is completing with position: x={self.robot_position.x}, y={self.robot_position.y}, z={self.robot_position.z}")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MoveToPointSkill is stopping with canceling the goal")
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
        self.node.get_logger().info("MoveToPointSkill is aborting")
        # Implement hard stop logic for the manipulator here

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MoveToPointSkill is resetting")
        self._action_client.destroy()
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False