import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/rotate/rotate", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/rotate/rotate", 
       description="Rotate the mobile robot by the desired number of degrees.")
class RotateSkill(ROS2Skill):

    MAX_ANGULAR_VELOCITY = 0.5  # Maximum angular velocity in rad/s

    def __init__(self):
        super().__init__('rotate')

        self.velocity_publisher = None
        self.velocity_cmd = Twist()
        self.current_orientation = 0.0
        self.target_orientation = 0.0
        self.odom_subscription = None
        self.received_odom = False

    @skill_parameter(is_required=True, name="desired_degrees", description="The desired rotation in degrees.")
    def get_desired_degrees(self) -> float:
        return math.degrees(self.target_orientation)

    @skill_output(is_required=True, name="current_orientation", description="The current orientation of the robot in degrees.")
    def get_current_orientation(self) -> float:
        return math.degrees(self.current_orientation)

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("RotateSkill is starting with creating a publisher to cmd_vel topic")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("RotateSkill is executing")
        self.target_orientation = self.current_orientation + math.radians(self.get_desired_degrees())
        self.target_orientation = self.target_orientation % (2 * math.pi)

        angular_velocity = self.MAX_ANGULAR_VELOCITY if self.get_desired_degrees() > 0 else -self.MAX_ANGULAR_VELOCITY
        self.velocity_cmd.angular.z = angular_velocity

        while not self.is_orientation_reached():
            self.velocity_publisher.publish(self.velocity_cmd)
            self.node.get_logger().info(f"Rotating with angular velocity: {self.velocity_cmd.angular.z}")
            time.sleep(0.1)

        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info("Desired orientation reached, stopping rotation.")

    def is_orientation_reached(self) -> bool:
        return abs(self.current_orientation - self.target_orientation) < math.radians(1.0)

    def odom_listener_callback(self, msg: Odometry) -> None:
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)
        self.received_odom = True
        self.node.get_logger().info(f'Received orientation: {math.degrees(self.current_orientation)} degrees')

    def quaternion_to_euler(self, q) -> float:
        # Convert quaternion to euler yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"RotateSkill is completing with orientation: {math.degrees(self.current_orientation)} degrees")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("RotateSkill is stopping")
        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info("Stopping robot by setting angular velocity to zero.")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("RotateSkill is resetting")
        self.velocity_publisher.destroy()
        self.odom_subscription.destroy()
        self.received_odom = False