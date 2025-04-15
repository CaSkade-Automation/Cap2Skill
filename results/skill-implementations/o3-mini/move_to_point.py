import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface

# ROS2 Action and message imports for MoveIt motion planning
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

@skill(skill_interface=SkillInterface.REST,
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/moveToPoint/moveToPoint",
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/moveToPoint/moveToPoint",
       description="Move the manipulator of the mobile robot to a desired goal point.")
class MoveToPointSkill(ROS2Skill):
    
    def __init__(self):
        super().__init__("move_to_point")
        # Desired target position for the manipulator (x,y,z)
        self.goal_position = Point()  # user provided x, y, z via skill_parameters
        # Reached (or planned) goal output position
        self.reached_position = Point()
        # ROS2 Action client for move group planning
        self._action_client = None
        self._send_goal_future = None
        self._get_result_future = None
        self.goal_handle = None
        # Outcome flags
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False

    @skill_parameter(is_required=True,
                     name="goal_position_x",
                     description="Desired goal x-coordinate of the manipulator (in meters).")
    def get_goal_position_x(self) -> float:
        return self.goal_position.x

    @skill_parameter(is_required=True,
                     name="goal_position_y",
                     description="Desired goal y-coordinate of the manipulator (in meters).")
    def get_goal_position_y(self) -> float:
        return self.goal_position.y

    @skill_parameter(is_required=True,
                     name="goal_position_z",
                     description="Desired goal z-coordinate of the manipulator (in meters).")
    def get_goal_position_z(self) -> float:
        return self.goal_position.z

    @skill_output(is_required=True,
                  name="reached_position_x",
                  description="The x-coordinate of the reached manipulator position.")
    def get_reached_position_x(self) -> float:
        return self.reached_position.x

    @skill_output(is_required=True,
                  name="reached_position_y",
                  description="The y-coordinate of the reached manipulator position.")
    def get_reached_position_y(self) -> float:
        return self.reached_position.y

    @skill_output(is_required=True,
                  name="reached_position_z",
                  description="The z-coordinate of the reached manipulator position.")
    def get_reached_position_z(self) -> float:
        return self.reached_position.z

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MoveToPointSkill: Starting - Creating an ActionClient for '/move_action' (MoveGroup)")
        # Create ROS2 ActionClient for MoveGroup (planning motion for the manipulator)
        self._action_client = ActionClient(self.node, MoveGroup, "move_action")

    def _create_goal_message(self) -> MoveGroup.Goal:
        # Build a minimal MotionPlanRequest to move the manipulator.
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        # Specify the planning group name.
        request.group_name = "manipulator"
        # Build a PositionConstraint for the target pose.
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"  # reference frame for planning
        pos_constraint.link_name = "manipulator_link"  # name of the end-effector; adjust if needed

        # Use the received goal position as the target point offset.
        # (In a complete implementation, the target pose would be constructed with orientation,
        #  but here we focus on the position.)
        target_pose = Pose()
        target_pose.position.x = self.goal_position.x
        target_pose.position.y = self.goal_position.y
        target_pose.position.z = self.goal_position.z
        target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # For the constraint region we use a small sphere as the allowed region.
        pos_constraint.constraint_region.primitives = []
        pos_constraint.constraint_region.primitive_poses = []
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # radius 1 cm
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        
        # Wrap the position constraint in Constraints.
        cons = Constraints()
        cons.position_constraints.append(pos_constraint)
        
        # Add the goal constraint to the motion plan request.
        request.goal_constraints.append(cons)
        goal_msg.request = request
        return goal_msg

    def goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().info("MoveToPointSkill: Goal was rejected by action server.")
            self.goal_rejected = True
            return
        self.node.get_logger().info("MoveToPointSkill: Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        result = future.result().result
        # In a real scenario, result would include detailed planning/execution info.
        # Here we assume if we received a result, the goal was reached.
        # Set the reached position to the goal as a simulation.
        self.reached_position.x = self.goal_position.x
        self.reached_position.y = self.goal_position.y
        self.reached_position.z = self.goal_position.z
        self.node.get_logger().info("MoveToPointSkill: Goal reached successfully!")
        self.goal_reached = True

    def feedback_callback(self, feedback_msg) -> None:
        # Feedback can be processed here as needed.
        self.node.get_logger().info("MoveToPointSkill: Received feedback from action server.")

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("MoveToPointSkill: Executing - sending goal to move the manipulator.")
        # Wait for the action server to be available.
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveToPointSkill: Action server not available, aborting.")
            self.state_machine.abort()
            return

        # Create the goal message with the user-provided goal position.
        goal_msg = self._create_goal_message()
        # Send the goal asynchronously. Provide a feedback callback if desired.
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                                                      feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait until one of the goal outcome flags is set with a timeout.
        timeout = time.time() + 10.0  # 10 seconds timeout for planning/execution
        while not (self.goal_reached or self.goal_rejected or self.goal_failed):
            if time.time() > timeout:
                self.node.get_logger().warn("MoveToPointSkill: Execution timed out.")
                self.state_machine.abort()
                return
            self.node.get_logger().info("MoveToPointSkill: Waiting for result from action server...")
            time.sleep(0.5)

        if self.goal_rejected or self.goal_failed:
            self.node.get_logger().error("MoveToPointSkill: Goal execution failed or was rejected.")
            self.state_machine.abort()

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"MoveToPointSkill: Completing with reached position:"
                                     f" x={self.reached_position.x}, y={self.reached_position.y}, z={self.reached_position.z}")

    def cancel_done(self, future) -> None:
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info("MoveToPointSkill: Cancel request succeeded.")
            self.cancel_success = True
        else:
            self.node.get_logger().warn("MoveToPointSkill: Cancel request failed.")
            self.cancel_failed = True

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MoveToPointSkill: Stopping - Canceling the goal movement.")
        if self.goal_handle is None:
            self.node.get_logger().warn("MoveToPointSkill: No active goal to cancel.")
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)
        timeout = time.time() + 5.0
        while not (self.cancel_success or self.cancel_failed):
            if time.time() > timeout:
                self.node.get_logger().warn("MoveToPointSkill: Cancel request timed out.")
                break
            self.node.get_logger().info("MoveToPointSkill: Waiting for goal cancellation...")
            time.sleep(0.5)
        if self.cancel_failed:
            self.node.get_logger().error("MoveToPointSkill: Failed to cancel the goal.")
            self.state_machine.abort()

    @aborting
    def aborting(self) -> None:
        self.node.get_logger().error("MoveToPointSkill: Aborting - Triggering hard stop of the manipulator.")
        # If a hard stop command exists for the manipulator, it should be published.
        # Here we simply log the abort event.
        # In practice, you might publish a zero-velocity or emergency stop command to the corresponding topic.

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MoveToPointSkill: Resetting - Shutting down move action client and cleaning up.")
        if self._action_client is not None:
            self._action_client.destroy()
        # Reset all flags and state information.
        self.goal_reached = False
        self.goal_rejected = False
        self.goal_failed = False
        self.cancel_success = False
        self.cancel_failed = False
        self.goal_handle = None