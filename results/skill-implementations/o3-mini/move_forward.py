import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

@skill(
    skill_interface=SkillInterface.REST,
    skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/moveForward/moveForward",
    module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
    capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/moveForward/moveForward",
    description="Move the mobile robot forward a desired distance in a desired time. "
                "A calculated velocity is used to move the robot. If the calculated velocity exceeds the max allowed velocity, "
                "the max velocity is applied and the required travel time is recalculated."
)
class MoveForwardSkill(ROS2Skill):

    # Maximum allowed velocity in m/s (as specified in the ontology by velocity_ID_Req)
    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__("move_forward")
        self.publisher = None
        self.odom_subscription = None
        self.current_velocity = 0.0  # Latest velocity reading from odometry

        # Parameters to be filled by user input:
        self.desired_distance = 0.0  # in meters
        self.desired_time = 0.0      # in seconds

        # Outputs for reporting
        self.actual_traveled_distance = 0.0  # in meters
        self.actual_travel_time = 0.0        # in seconds

    @skill_parameter(is_required=True, name="desired_distance", description="The desired distance the robot should move forward (in meters).")
    def get_desired_distance(self) -> float:
        return self.desired_distance

    @skill_parameter(is_required=True, name="desired_time", description="The desired time to cover the given distance (in seconds).")
    def get_desired_time(self) -> float:
        return self.desired_time

    @skill_output(is_required=True, name="output_velocity", description="The current velocity of the robot (in m/s).")
    def get_output_velocity(self) -> float:
        return self.current_velocity

    @skill_output(is_required=True, name="traveled_distance", description="The distance actually traveled by the robot (in meters).")
    def get_traveled_distance(self) -> float:
        return self.actual_traveled_distance

    @skill_output(is_required=True, name="travel_time", description="The time taken by the robot to cover the distance (in seconds).")
    def get_travel_time(self) -> float:
        return self.actual_travel_time

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MoveForwardSkill starting: Setting up publisher on 'cmd_vel' and subscribing to 'odom'.")
        self.publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.odom_subscription = self.node.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

    def odom_callback(self, msg: Odometry) -> None:
        # Update current_velocity from odometry data for output reporting.
        self.current_velocity = msg.twist.twist.linear.x

    @execute
    def execute(self) -> None:
        # Calculate the required velocity to cover the desired distance in the desired time.
        if self.desired_time <= 0:
            self.node.get_logger().error("Desired time must be greater than zero.")
            return

        calculated_velocity = self.desired_distance / self.desired_time
        effective_time = self.desired_time

        # Check if calculated velocity exceeds max allowed.
        if calculated_velocity > self.MAX_VELOCITY:
            self.node.get_logger().warn(
                f"Calculated velocity ({calculated_velocity:.2f} m/s) exceeds maximum allowed velocity of {self.MAX_VELOCITY} m/s. "
                f"Setting velocity to maximum."
            )
            calculated_velocity = self.MAX_VELOCITY
            effective_time = self.desired_distance / self.MAX_VELOCITY
            self.node.get_logger().info(f"New effective travel time recalculated as {effective_time:.2f} seconds.")

        twist_cmd = Twist()
        twist_cmd.linear.x = calculated_velocity

        self.node.get_logger().info(
            f"Executing move_forward: Moving forward {self.desired_distance:.2f} m "
            f"in {effective_time:.2f} s at {calculated_velocity:.2f} m/s."
        )

        start_time = time.time()
        # Publish the velocity command continuously for the effective_time duration.
        while (time.time() - start_time) < effective_time:
            self.publisher.publish(twist_cmd)
            time.sleep(0.1)
        # Stop the robot by publishing zero velocity.
        twist_cmd.linear.x = 0.0
        self.publisher.publish(twist_cmd)

        self.actual_travel_time = time.time() - start_time
        self.actual_traveled_distance = calculated_velocity * self.actual_travel_time

        self.node.get_logger().info("Movement command executed; robot commanded to stop.")

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(
            f"MoveForwardSkill completing: Current velocity = {self.current_velocity:.2f} m/s, "
            f"Traveled distance = {self.actual_traveled_distance:.2f} m, "
            f"Travel time = {self.actual_travel_time:.2f} s."
        )

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MoveForwardSkill stopping: Setting robot velocity to zero.")
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        self.publisher.publish(twist_cmd)

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MoveForwardSkill resetting: Shutting down publishers and subscriptions.")
        self.publisher.destroy()
        self.odom_subscription.destroy()