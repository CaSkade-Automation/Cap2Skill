from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import *

from pyskillup.decorators.skill_interface import SkillInterface

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

@skill(skill_interface = SkillInterface.REST, skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/skills/get-position", module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/skills/get-position", capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots#neobotix-mmo-700/capabilities/get-position", description = "Retrieve the current position of the mobile robot.")
class GetPositionSkill(Node): 

    def __init__(self):
        super().__init__('get_position')
        self.position = Point()

    @skill_output(is_required=True, name="position", description="The position of the robot.")
    def get_position(self) -> Point:
        return self.position

    @starting
    def starting(self) -> None:
        self.get_logger().info("GetPositionSkill is starting with subscription to odom topic")
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)

    def odom_listener_callback(self, msg: Odometry) -> None: 
        self.position = msg.pose.pose.position
        self.get_logger().info(f'Position: x={self.position.x}, y={self.position.y}')

    @execute
    def execute(self) -> Point: 
        self.get_logger().info(f'Position: x={self.position.x}, y={self.position.y}')
        return self.position
    
    @completing
    def stopping(self) -> None:
        self.get_logger().info("GetPositionSkill is completing")
        self.odom_subscription.destroy()

def main(args=None):
    rclpy.init(args=args)

    get_position_skill = GetPositionSkill()

    rclpy.spin(get_position_skill)

    get_position_skill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()