import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/collisionAvoidance/collisionAvoidance", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/collisionAvoidance/collisionAvoidance", 
       description="Move the mobile robot with the desired velocity in a desired time while detecting and avoiding obstacles.")
class CollisionAvoidanceSkill(ROS2Skill):

    def __init__(self):
        super().__init__("collision_avoidance")

        self.velocity_cmd = Twist()
        self.velocity_publisher = None
        self.laser_subscription = None
        self.min_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.start_time = None
        self.desired_time = 0.0
        self.obstacle_threshold = 0.5  # meters

    @skill_parameter(is_required=True, name="velocity_x", description="The desired forward velocity of the robot.")
    def get_velocity_x(self) -> float:
        return self.velocity_cmd.linear.x

    @skill_parameter(is_required=True, name="velocity_y", description="The desired sideways velocity of the robot.")
    def get_velocity_y(self) -> float:
        return self.velocity_cmd.linear.y

    @skill_parameter(is_required=True, name="angular_velocity", description="The desired angular velocity of the robot.")
    def get_angular_velocity(self) -> float:
        return self.velocity_cmd.angular.z

    @skill_parameter(is_required=True, name="desired_time", description="The desired time to move the robot.")
    def get_desired_time(self) -> float:
        return self.desired_time

    @skill_output(is_required=True, name="current_velocity_x", description="The current forward velocity of the robot.")
    def get_current_velocity_x(self) -> float:
        return self.velocity_cmd.linear.x

    @skill_output(is_required=True, name="current_velocity_y", description="The current sideways velocity of the robot.")
    def get_current_velocity_y(self) -> float:
        return self.velocity_cmd.linear.y

    @skill_output(is_required=True, name="current_angular_velocity", description="The current angular velocity of the robot.")
    def get_current_angular_velocity(self) -> float:
        return self.velocity_cmd.angular.z

    @skill_output(is_required=True, name="traveled_time", description="The time the robot has been moving.")
    def get_traveled_time(self) -> float:
        return time.time() - self.start_time if self.start_time else 0.0

    @skill_output(is_required=True, name="nearest_obstacle_distance", description="The distance to the nearest obstacle.")
    def get_nearest_obstacle_distance(self) -> float:
        return self.min_obstacle_distance

    @skill_output(is_required=True, name="nearest_obstacle_angle", description="The angle to the nearest obstacle.")
    def get_nearest_obstacle_angle(self) -> float:
        return self.obstacle_angle

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is starting...")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscription = self.node.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.start_time = time.time()

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is executing...")
        while time.time() - self.start_time < self.desired_time:
            if self.min_obstacle_distance < self.obstacle_threshold:
                self.node.get_logger().warn("Obstacle detected! Stopping the robot.")
                self.state_machine.stop()
                return

            self.velocity_publisher.publish(self.velocity_cmd)
            self.node.get_logger().info(f"Publishing velocity: linear_x={self.velocity_cmd.linear.x}, linear_y={self.velocity_cmd.linear.y}, angular_z={self.velocity_cmd.angular.z}")
            time.sleep(0.1)

        self.node.get_logger().info("Desired time elapsed. Stopping the robot.")
        self.state_machine.stop()

    def laser_callback(self, msg: LaserScan) -> None:
        self.min_obstacle_distance = min(msg.ranges)
        self.obstacle_angle = msg.ranges.index(self.min_obstacle_distance) * msg.angle_increment
        self.node.get_logger().info(f"Nearest obstacle at distance: {self.min_obstacle_distance}, angle: {self.obstacle_angle}")

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"CollisionAvoidanceSkill is completing with velocity: linear_x={self.velocity_cmd.linear.x}, linear_y={self.velocity_cmd.linear.y}, angular_z={self.velocity_cmd.angular.z}, traveled_time={self.get_traveled_time()}, nearest_obstacle_distance={self.min_obstacle_distance}, nearest_obstacle_angle={self.obstacle_angle}")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is stopping...")
        self.velocity_cmd.linear.x = 0.0
        self.velocity_cmd.linear.y = 0.0
        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info("Robot stopped.")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("CollisionAvoidanceSkill is resetting...")
        self.velocity_publisher.destroy()
        self.laser_subscription.destroy()
        self.min_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.start_time = None