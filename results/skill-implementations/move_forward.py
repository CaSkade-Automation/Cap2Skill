import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/moveForward/moveForward", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/moveForward/moveForward", 
       description="Move the mobile robot forward a desired distance in a desired time.")
class MoveForwardSkill(ROS2Skill):

    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__('move_forward')

        self.velocity_publisher = None
        self.velocity_cmd = Twist()
        self.velocity_robot = Twist()
        self.odom_subscription = None
        self.received_odom = False
        self.start_time = None
        self.traveled_distance = 0.0

    @skill_parameter(is_required=True, name="desired_distance", description="The desired distance to move the robot forward.")
    def get_desired_distance(self) -> float:
        return self.desired_distance

    @skill_parameter(is_required=True, name="desired_time", description="The desired time to move the robot forward.")
    def get_desired_time(self) -> float:
        return self.desired_time

    @skill_output(is_required=True, name="output_velocity", description="The current velocity of the robot.")
    def get_output_velocity(self) -> float:
        return self.velocity_robot.linear.x

    @skill_output(is_required=True, name="traveled_distance", description="The distance traveled by the robot.")
    def get_traveled_distance(self) -> float:
        return self.traveled_distance

    @skill_output(is_required=True, name="elapsed_time", description="The time elapsed during the movement.")
    def get_elapsed_time(self) -> float:
        return time.time() - self.start_time if self.start_time else 0.0

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MoveForwardSkill is starting with creating a publisher to cmd_vel topic")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)
        self.start_time = time.time()

    @execute
    def execute(self) -> None:
        desired_velocity = self.desired_distance / self.desired_time
        if desired_velocity > self.MAX_VELOCITY:
            self.node.get_logger().warn(f"Desired velocity {desired_velocity} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            desired_velocity = self.MAX_VELOCITY
            self.desired_time = self.desired_distance / desired_velocity

        self.velocity_cmd.linear.x = desired_velocity
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')

        time.sleep(self.desired_time)
        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)

    def odom_listener_callback(self, msg: Odometry) -> None:
        self.velocity_robot = msg.twist.twist
        self.traveled_distance += self.velocity_robot.linear.x * (time.time() - self.start_time)
        self.start_time = time.time()
        self.received_odom = True

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"MoveForwardSkill is completing with velocity: {self.velocity_robot.linear.x}")
        self.node.get_logger().info(f"Traveled distance: {self.traveled_distance}, Elapsed time: {self.get_elapsed_time()}")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MoveForwardSkill is stopping")
        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Stopping robot by publishing velocity: {self.velocity_cmd.linear.x}')

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MoveForwardSkill is resetting")
        self.velocity_publisher.destroy()
        self.odom_subscription.destroy()
        self.received_odom = False
        self.start_time = None
        self.traveled_distance = 0.0