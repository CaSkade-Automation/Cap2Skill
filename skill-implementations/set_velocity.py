from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/skills/set-velocity", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/capabilities/set-velocity", description = "Set the velocity of the mobile robot in the forward direction.")
class SetVelocitySkill(ROS2Skill): 

    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__('set_velocity')

        self.velocity_publisher = None
        self.velocity_cmd = Twist()

    @skill_parameter(is_required=True, name="velocity", description="The desired velocity of the robot.")
    def get_velocity(self) -> float:
        return self.velocity_cmd.linear.x

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("SetVelocitySkill is starting with creating a publisher to cmd_vel topic")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    @execute
    def execute(self) -> None: 
        if self.velocity_cmd.linear.x > self.MAX_VELOCITY:
            self.node.get_logger().warn(f"Desired velocity {self.velocity_cmd.linear.x} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            self.velocity_cmd.linear.x = self.MAX_VELOCITY

        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("SetVelocitySkill is stopping")
        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Stopping robot by publishing velocity: {self.velocity_cmd.linear.x}')

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("SetVelocitySkill is resetting")
        self.velocity_publisher.destroy()