import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/moveForward/moveForward", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/moveForward/moveForward", description = "Move the mobile robot forward a desired distance in a desired time.")
class MoveForwardSkill(ROS2Skill): 

    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__('move_forward')
        self.velocity_cmd = Twist()
        self.velocity_publisher = None
        self.distance = 0.0
        self.duration = 0.0
        self.distance_traveled = 0.0
        self.time_needed = 0.0
        self.robot_velocity = Twist()
        self.received_odom = False

    @skill_parameter(is_required=True, name="distance", description="The distance to move forward in meters.")
    def get_distance(self) -> float:
        return self.distance
    
    @skill_parameter(is_required=True, name="duration", description="The time to move forward in seconds.")
    def get_duration(self) -> float:
        return self.duration
    
    @skill_output(is_required=True, name="traveled_distance", description="The actual distance traveled in meters.")
    def get_traveled_distance(self) -> float:
        return self.distance_traveled
    
    @skill_output(is_required=True, name="time_needed", description="The actual time needed to move forward in seconds.")
    def get_time_needed(self) -> float:
        return self.time_needed
    
    @skill_output(is_required=True, name="robot_velocity", description="The robot velocity in m/s.")
    def get_robot_velocity(self) -> float:
        return self.robot_velocity.linear.x
    
    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MoveForwardSkill is starting.")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    @execute
    def execute(self) -> None: 
        self.node.get_logger().info("MoveForwardSkill is executing.")
        velocity = self.distance / self.duration
        if velocity > self.MAX_VELOCITY:
            self.node.get_logger().warn(f"Calculated velocity {velocity} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            velocity = self.MAX_VELOCITY
            self.duration = self.distance / self.MAX_VELOCITY
            self.node.get_logger().warn(f"New duration is {self.duration} seconds.")

        self.velocity_cmd.linear.x = velocity

        start_time = time.time()

        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')

        time.sleep(self.duration)

        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.node.get_logger().info(f'Movement finished.')

        end_time = time.time()
        self.time_needed = end_time - start_time

        self.distance_traveled = velocity * self.time_needed

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"Distance traveled: {self.distance_traveled:.2f}m")
        self.node.get_logger().info(f"Time needed: {self.time_needed:.2f}s")

        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)
        while not self.received_odom:
            self.node.get_logger().info("Waiting for odometry data...")
            time.sleep(0.1)
        
        self.node.get_logger().info(f'Robot velocity: {self.robot_velocity.linear.x}')

    def odom_listener_callback(self, msg: Odometry) -> None: 
        self.robot_velocity = msg.twist.twist
        self.received_odom = True
        self.node.get_logger().info(f'Received velocity: {self.robot_velocity.linear.x}')

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