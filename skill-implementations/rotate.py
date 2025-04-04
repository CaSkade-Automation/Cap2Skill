import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/rotate/rotate", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/rotate/rotate", description = "Rotate the mobile robot to a desired orientation.")
class RotateSkill(ROS2Skill): 

    MAX_VELOCITY = 0.5  

    def __init__(self):
        super().__init__('rotate')
        self.target_degrees: float = None
        self.target_angular_speed: float = None
        self.robot_final_orientation: float = None

        self.initial_yaw: float = None
        self.current_yaw: float = None
        self.received_odom = False

        self.velocity_cmd = Twist()
        self.velocity_publisher = None
        self.odom_subscription = None
        self.timer = None
        self.rotation_done = False

    @skill_parameter(is_required=True, name="target_degrees", description="The target angle in degrees to rotate.")
    def get_target_degrees(self) -> float:
        return self.target_degrees
    
    @skill_parameter(is_required=True, name="target_angular_speed", description="The target angular speed in degrees per second.")
    def get_target_angular_speed(self) -> float:
        return self.target_angular_speed
    
    @skill_output(is_required=True, name="robot_orientation", description="The current yaw angle of the robot in radians.")
    def get_robot_final_orientation(self) -> float:
        return self.robot_final_orientation

    def shortest_angular_distance(self, from_angle, to_angle):
        return math.atan2(math.sin(to_angle - from_angle), math.cos(to_angle - from_angle))

    def control_loop(self):
        if not self.received_odom or self.current_yaw is None:
            return

        self.node.get_logger().info(f'Starting rotation: Target = {self.target_degrees} degrees')

        target_radians = math.radians(self.target_degrees)
        rotated_angle = self.shortest_angular_distance(self.initial_yaw, self.current_yaw)

        if self.target_angular_speed > self.MAX_VELOCITY:
            self.node.get_logger().warn(f"Desired velocity {self.target_angular_speed} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            self.target_angular_speed = self.MAX_VELOCITY

        if abs(rotated_angle) < abs(target_radians):
            self.velocity_cmd.angular.z = self.target_angular_speed if target_radians > 0 else -self.target_angular_speed
            self.velocity_publisher.publish(self.velocity_cmd)
        else:
            self.velocity_cmd.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_cmd)
            self.robot_final_orientation = math.degrees(self.current_yaw)
            self.node.get_logger().info('Rotation complete.')
            self.timer.cancel()
            self.rotation_done = True
    
    @starting
    def starting(self) -> None:
        self.node.get_logger().info("RotateSkill is starting.")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    @execute
    def execute(self) -> None: 
        self.node.get_logger().info("RotateSkill is executing.")
        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)

        self.timer = self.node.create_timer(0.1, self.control_loop)

        while not self.rotation_done:
            time.sleep(1)

    def odom_listener_callback(self, msg: Odometry) -> None:
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw
        self.received_odom = True

        if self.initial_yaw is None:
            self.initial_yaw = yaw


    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"RotateSkill is completing with final orientation: {self.robot_final_orientation:.2f} degrees")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("RotateSkill is stopping")
        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Stopping robot by publishing velocity: {self.velocity_cmd.angular.z}')

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("RotateSkill is resetting")
        self.node.destroy_publisher(self.velocity_publisher)
        self.node.destroy_subscription(self.odom_subscription)
        self.velocity_publisher = None
        self.odom_subscription = None
        self.received_odom = False
        self.current_yaw = None
        self.rotation_done = False
        self.initial_yaw = None