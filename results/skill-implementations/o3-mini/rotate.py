import time
import math
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

@skill(
    skill_interface=SkillInterface.REST,
    skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/rotate/rotate",
    module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
    capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/rotate/rotate",
    description="Rotate the mobile robot by the desired number of degrees."
)
class RotateSkill(ROS2Skill):
    # Maximum allowed angular velocity (from the ontology constraint)
    MAX_ANGULAR_VELOCITY = 0.5  # m/s (as specified by the inVelocity requirement)

    def __init__(self):
        super().__init__("rotate")
        # Publisher for command velocities
        self.cmd_vel_publisher = None
        # Subscription handle for odometry
        self.odom_subscription = None

        # Current yaw angle of robot (in radians)
        self.current_yaw = None
        # Yaw angle at the start of rotation (in radians)
        self.starting_yaw = None

        # Tolerance for stopping the rotation (2 degrees in radians)
        self.angle_tolerance = math.radians(2)

    @skill_parameter(is_required=True, name="desired_degree", 
                     description="The desired rotation in degrees. "
                                 "Positive values correspond to counter-clockwise rotation and negative to clockwise.")
    def get_desired_degree(self) -> float:
        # This parameter must be provided externally.
        return self.desired_degree

    @skill_parameter(is_required=True, name="desired_angular_velocity", 
                     description="The desired angular velocity (in rad/s) for rotating the robot. Maximum allowed is 0.5.")
    def get_desired_angular_velocity(self) -> float:
        # This parameter must be provided externally.
        return self.desired_angular_velocity

    @skill_output(is_required=True, name="final_orientation", 
                  description="The final orientation (yaw in degrees) of the robot after rotation.")
    def get_final_orientation(self) -> float:
        if self.current_yaw is not None:
            # Convert current yaw from radians to degrees
            return math.degrees(self.current_yaw)
        return 0.0

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("RotateSkill is starting: setting up publisher and odometry subscription.")
        # Create a publisher to the cmd_vel topic
        self.cmd_vel_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        # Create subscription to the odometry topic to receive the robot's orientation updates
        self.odom_subscription = self.node.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        # Reset initial yaw
        self.starting_yaw = None

    def odom_callback(self, msg: Odometry) -> None:
        # Extract quaternion from odometry pose and convert to yaw (rotation about z-axis)
        q = msg.pose.pose.orientation
        # Compute yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

        # Set the starting yaw, if not already done.
        if self.starting_yaw is None:
            self.starting_yaw = yaw
            self.node.get_logger().info(f"Initial orientation captured: {math.degrees(yaw):.2f}°")

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("RotateSkill is executing.")
        
        # Wait until the first odometry message has been received
        wait_time = 0.0
        while self.current_yaw is None and wait_time < 5.0:
            self.node.get_logger().info("Waiting for odometry data...")
            time.sleep(0.1)
            wait_time += 0.1
        if self.current_yaw is None:
            self.node.get_logger().error("Odometry data could not be received in time.")
            self.state_machine.abort()
            return

        # Retrieve parameter values for desired degree and angular velocity
        desired_degree = self.get_desired_degree()
        desired_angular_velocity = self.get_desired_angular_velocity()

        # Enforce the maximum allowed angular velocity constraint.
        if abs(desired_angular_velocity) > self.MAX_ANGULAR_VELOCITY:
            self.node.get_logger().warn(
                f"Desired angular velocity {desired_angular_velocity} exceeds maximum {self.MAX_ANGULAR_VELOCITY}. Limiting it."
            )
            desired_angular_velocity = math.copysign(self.MAX_ANGULAR_VELOCITY, desired_angular_velocity)

        self.node.get_logger().info(
            f"Rotating robot by {desired_degree} degrees with angular velocity {desired_angular_velocity} rad/s."
        )

        # Compute target orientation in radians.
        # Convert desired rotation from degrees to radians.
        delta_angle_rad = math.radians(desired_degree)
        target_yaw = normalize_angle(self.starting_yaw + delta_angle_rad)
        self.node.get_logger().info(f"Target orientation (yaw): {math.degrees(target_yaw):.2f}°")

        rate = 0.1  # seconds
        twist_msg = Twist()
        # Continue publishing until target is reached
        while True:
            if self.current_yaw is None:
                self.node.get_logger().warn("No current orientation received; continuing to wait.")
                time.sleep(rate)
                continue

            # Compute angular error between target and current orientation
            error = normalize_angle(target_yaw - self.current_yaw)
            self.node.get_logger().info(f"Current yaw: {math.degrees(self.current_yaw):.2f}°, Error: {math.degrees(error):.2f}°")
            if abs(error) < self.angle_tolerance:
                self.node.get_logger().info("Desired orientation reached within tolerance.")
                break

            # Set the angular velocity command in the direction of the error
            twist_msg.angular.z = math.copysign(desired_angular_velocity, error)
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0

            self.cmd_vel_publisher.publish(twist_msg)
            time.sleep(rate)

        # Stop the rotation by sending zero angular velocity.
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    @completing
    def completing(self) -> None:
        final_orientation_deg = math.degrees(self.current_yaw) if self.current_yaw is not None else 0.0
        self.node.get_logger().info(f"RotateSkill is completing. Final orientation: {final_orientation_deg:.2f}°")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("RotateSkill is stopping. Sending zero velocity command.")
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("RotateSkill is resetting. Cleaning up publishers and subscriptions.")
        if self.cmd_vel_publisher is not None:
            self.cmd_vel_publisher.destroy()
        if self.odom_subscription is not None:
            self.node.destroy_subscription(self.odom_subscription)
        self.current_yaw = None
        self.starting_yaw = None