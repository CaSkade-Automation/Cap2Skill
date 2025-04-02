import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/skills/get-position", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/capabilities/get-position", description = "Retrieve the current position of the mobile robot.")
class GetPositionSkill(ROS2Skill): 

    def __init__(self):
        super().__init__('get_position')

        self.position = Point()
        self.odom_subscription = None
        self.received_odom = False

    @skill_output(is_required=True, name="position_x", description="The x coordinate of the robot's position.")
    def get_position_x(self) -> float:
        return self.position.x
    
    @skill_output(is_required=True, name="position_y", description="The y coordinate of the robot's position.")
    def get_position_y(self) -> float:
        return self.position.y

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("GetPositionSkill is starting...")

    @execute
    def execute(self) -> None: 
        self.node.get_logger().info("GetPositionSkill is executing with subscription to odometry data.")
        self.odom_subscription = self.node.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)
        while not self.received_odom:
            self.node.get_logger().info("Waiting for odometry data...")
            time.sleep(0.1)

    def odom_listener_callback(self, msg: Odometry) -> None: 
        self.position = msg.pose.pose.position
        self.received_odom = True
        self.node.get_logger().info(f'Received Position: x={self.position.x}, y={self.position.y}')

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"GetPositionSkill is completing with position: x={self.position.x}, y={self.position.y}")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("GetPositionSkill is resetting")
        self.odom_subscription.destroy()
        self.received_odom = False