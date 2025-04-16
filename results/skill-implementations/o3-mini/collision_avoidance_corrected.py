import time
import math
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

@skill(
    skill_interface=SkillInterface.REST,
    skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/collisionAvoidance/collisionAvoidance",
    module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
    capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/collisionAvoidance/collisionAvoidance",
    description="Move the mobile robot with the desired velocity in a desired time while detecting and avoiding obstacles."
)
class CollisionAvoidanceSkill(ROS2Skill):
    def __init__(self):
        super().__init__("collision_avoidance")
        # Publisher for velocity commands on /cmd_vel
        self.vel_pub = None
        # Subscriber for LiDAR scan data on /scan topic
        self.scan_sub = None

        # Velocity command to publish
        self.vel_cmd = Twist()

        # Variables to store sensor data and timing
        self.closest_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.start_time = 0.0
        self.travel_time = 0.0

        # Variables to store current applied velocities
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_angular_velocity = 0.0

    # Skill Parameters: these are supposed to be externally provided by the skill user.
    @skill_parameter(is_required=True, name="desired_velocity_x", 
                      description="Desired linear velocity in the x-direction [m/s].")
    def get_desired_velocity_x(self) -> float:
        return self.desired_velocity_x

    @skill_parameter(is_required=True, name="desired_velocity_y", 
                      description="Desired linear velocity in the y-direction [m/s].")
    def get_desired_velocity_y(self) -> float:
        return self.desired_velocity_y

    @skill_parameter(is_required=True, name="desired_angular_velocity", 
                      description="Desired angular velocity [rad/s].")
    def get_desired_angular_velocity(self) -> float:
        return self.desired_angular_velocity

    @skill_parameter(is_required=True, name="desired_time", 
                      description="Desired duration to move with the given velocities in seconds.")
    def get_desired_time(self) -> float:
        return self.desired_time

    @skill_parameter(is_required=True, name="min_obstacle_distance", 
                      description="Minimum allowed distance to an obstacle [m]. If an obstacle is closer, the robot stops.")
    def get_min_obstacle_distance(self) -> float:
        return self.min_obstacle_distance

    # Skill Outputs: these provide information about the executed motion.
    @skill_output(is_required=True, name="current_velocity_x", 
                  description="The current linear velocity in the x-direction [m/s].")
    def get_current_velocity_x(self) -> float:
        return self.current_velocity_x

    @skill_output(is_required=True, name="current_velocity_y", 
                  description="The current linear velocity in the y-direction [m/s].")
    def get_current_velocity_y(self) -> float:
        return self.current_velocity_y

    @skill_output(is_required=True, name="current_angular_velocity", 
                  description="The current angular velocity [rad/s].")
    def get_current_angular_velocity(self) -> float:
        return self.current_angular_velocity

    @skill_output(is_required=True, name="travel_time", 
                  description="The total time the robot moved [s].")
    def get_travel_time(self) -> float:
        return self.travel_time

    @skill_output(is_required=True, name="obstacle_distance", 
                  description="Distance to the nearest detected obstacle [m].")
    def get_obstacle_distance(self) -> float:
        return self.closest_obstacle_distance

    @skill_output(is_required=True, name="obstacle_degree", 
                  description="The angle (in degrees) at which the closest obstacle was detected.")
    def get_obstacle_degree(self) -> float:
        return self.obstacle_angle

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is starting: setting up publishers and subscribers.")
        # Create publisher for velocity commands on /cmd_vel
        self.vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribe to LiDAR sensor messages on /scan
        self.scan_sub = self.node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Initialize timing variables
        self.start_time = 0.0
        self.travel_time = 0.0

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is executing:")
        # Set desired velocities from parameters
        self.vel_cmd.linear.x = self.get_desired_velocity_x()
        self.vel_cmd.linear.y = self.get_desired_velocity_y()
        self.vel_cmd.angular.z = self.get_desired_angular_velocity()

        desired_duration = self.get_desired_time()
        min_obst_dist = self.get_min_obstacle_distance()

        # Record the start time
        self.start_time = time.time()
        elapsed = 0.0
        
        # Continuously publish velocity command until desired time elapses
        # or an obstacle is closer than the minimum threshold.
        while elapsed < desired_duration:
            # Check obstacle distance from sensor data (if available)
            if self.closest_obstacle_distance < min_obst_dist:
                self.node.get_logger().warn(f"Obstacle detected at {self.closest_obstacle_distance:.2f} m which is below threshold {min_obst_dist} m. Stopping motion.")
                self.state_machine.stop()
                break

            # Publish the velocity command
            self.vel_pub.publish(self.vel_cmd)
            self.current_velocity_x = self.vel_cmd.linear.x
            self.current_velocity_y = self.vel_cmd.linear.y
            self.current_angular_velocity = self.vel_cmd.angular.z

            self.node.get_logger().info(f"Publishing velocity command: linear x={self.current_velocity_x:.2f}, y={self.current_velocity_y:.2f}, angular z={self.current_angular_velocity:.2f}")

            time.sleep(0.1)
            elapsed = time.time() - self.start_time

        # Update travel time
        self.travel_time = time.time() - self.start_time

    @completing
    def completing(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is completing.")
        self.node.get_logger().info(f"Final Velocity: linear x={self.current_velocity_x:.2f}, y={self.current_velocity_y:.2f}, angular z={self.current_angular_velocity:.2f}")
        self.node.get_logger().info(f"Traveled time: {self.travel_time:.2f} s")
        self.node.get_logger().info(f"Nearest obstacle: distance = {self.closest_obstacle_distance:.2f} m, angle = {self.obstacle_angle:.2f}°")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is stopping: sending zero velocity command.")
        # Publish a zero velocity command to stop the robot
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0
        self.vel_pub.publish(stop_cmd)
        # Update current velocities
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_angular_velocity = 0.0

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is resetting: cleaning up publishers and subscribers.")
        if self.vel_pub is not None:
            self.vel_pub.destroy()
        if self.scan_sub is not None:
            self.node.destroy_subscription(self.scan_sub)
        # Reset sensor and timer variables
        self.closest_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.start_time = 0.0
        self.travel_time = 0.0

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Callback to process LiDAR scan data.
        Updates the closest obstacle distance and its angle.
        """
        if not msg.ranges:
            return

        # Filter valid ranges
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return

        # Find the minimum range and its index
        min_range = min(valid_ranges)
        index = msg.ranges.index(min_range)
        # Calculate the angle in degrees (convert from radians)
        angle_rad = msg.angle_min + index * msg.angle_increment
        angle_deg = math.degrees(angle_rad)

        self.closest_obstacle_distance = min_range
        self.obstacle_angle = angle_deg
        self.node.get_logger().debug(f"Scan callback: closest obstacle at {min_range:.2f} m, angle {angle_deg:.2f}°")