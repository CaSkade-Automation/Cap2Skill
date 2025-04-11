import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/setVelocity/setVelocity", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/setVelocity/setVelocity", description = "Set the velocity of the mobile robot in the forward direction.")
class SetVelocitySkill(ROS2Skill): 

    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__('set_velocity')

        self.velocity_publisher = None
        self.velocity_cmd = Twist()
        self.velocity_robot = Twist()
        self.odom_subscription = None
        self.received_odom = False

    @skill_parameter(is_required=True, name="desired_velocity", description="The desired velocity of the robot.")
    def get_desired_velocity(self) -> float:
        return self.velocity_cmd.linear.x
    
    @skill_output(is_required=True, name="output_velocity", description="The current velocity of the robot.")
    def get_output_velocity(self) -> float:
        return self.velocity_robot.linear.x

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("SetVelocitySkill is starting with creating a publisher to cmd_vel topic")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)

    @execute
    def execute(self) -> None: 
        if self.velocity_cmd.linear.x > self.MAX_VELOCITY:
            self.node.get_logger().warn(f"Desired velocity {self.velocity_cmd.linear.x} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            self.velocity_cmd.linear.x = self.MAX_VELOCITY

        timeout = time.time() + 3.0 
        velocity_tolerance = 0.05   

        while time.time() < timeout:
            self.velocity_publisher.publish(self.velocity_cmd)
            self.node.get_logger().info(f"Publishing velocity: {self.velocity_cmd.linear.x}")

            if abs(self.velocity_robot.linear.x - self.velocity_cmd.linear.x) <= velocity_tolerance:
                self.node.get_logger().info(f"Desired velocity reached: {self.velocity_robot.linear.x}")
                break

            time.sleep(0.1)

        else:
            self.node.get_logger().warn(f"Velocity not reached within timeout. Last received: {self.velocity_robot.linear.x}")

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"SetVelocitySkill is completing with velocity: {self.velocity_robot.linear.x}")

    def odom_listener_callback(self, msg: Odometry) -> None: 
        self.velocity_robot = msg.twist.twist
        self.received_odom = True
        self.node.get_logger().info(f'Received velocity: {self.velocity_robot.linear.x}')

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
        self.odom_subscription.destroy()
        self.received_odom = False