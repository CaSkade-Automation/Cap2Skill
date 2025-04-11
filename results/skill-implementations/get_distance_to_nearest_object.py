import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from sensor_msgs.msg import LaserScan
import numpy as np

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/getDistanceToNearestObject/getDistanceToNearestObject", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/getDistanceToNearestObject/getDistanceToNearestObject", 
       description="Retrieve the distance and orientation to the mobile robot's nearest object using LiDAR sensor data.")
class GetDistanceToNearestObjectSkill(ROS2Skill):

    def __init__(self):
        super().__init__('get_distance_to_nearest_object')

        self.nearest_distance = float('inf')
        self.nearest_angle = 0.0
        self.scan_subscription = None
        self.received_scan = False

    @skill_output(is_required=True, name="nearest_distance", description="The distance to the nearest object.")
    def get_nearest_distance(self) -> float:
        return self.nearest_distance
    
    @skill_output(is_required=True, name="nearest_angle", description="The angle to the nearest object.")
    def get_nearest_angle(self) -> float:
        return self.nearest_angle

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill is starting...")
        self.scan_subscription = self.node.create_subscription(LaserScan, "scan", self.scan_listener_callback, 10)

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill is executing with subscription to LiDAR data.")
        while not self.received_scan:
            self.node.get_logger().info("Waiting for LiDAR data...")
            time.sleep(0.1)

    def scan_listener_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges)
        min_distance_index = np.argmin(ranges)
        self.nearest_distance = ranges[min_distance_index]
        self.nearest_angle = msg.angle_min + min_distance_index * msg.angle_increment
        self.node.get_logger().info(f'Received Nearest Object: distance={self.nearest_distance}, angle={self.nearest_angle}')
        self.received_scan = True

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"GetDistanceToNearestObjectSkill is completing with nearest object: distance={self.nearest_distance}, angle={self.nearest_angle}")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill is resetting")
        self.node.destroy_subscription(self.scan_subscription)
        self.received_scan = False