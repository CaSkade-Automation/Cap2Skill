import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface

from sensor_msgs.msg import LaserScan

@skill(
    skill_interface=SkillInterface.REST,
    skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/getDistanceToNearestObject/getDistanceToNearestObject",
    module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
    capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/getDistanceToNearestObject/getDistanceToNearestObject",
    description="Retrieve the distance and orientation to the mobile robot's nearest object using LiDAR sensor data"
)
class GetDistanceToNearestObjectSkill(ROS2Skill):
    def __init__(self):
        super().__init__("get_distance_to_nearest_object")
        self.laser_scan = None
        self.subscription = None
        self.nearest_distance = None
        self.nearest_angle = None

    @skill_output(is_required=True, name="distance", description="Distance to the nearest object in meters.")
    def get_distance(self) -> float:
        return self.nearest_distance if self.nearest_distance is not None else float('nan')

    @skill_output(is_required=True, name="degree", description="Orientation (in degrees) to the nearest object.")
    def get_degree(self) -> float:
        return self.nearest_angle if self.nearest_angle is not None else float('nan')

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill: Starting and subscribing to /scan for LiDAR data.")
        # Create a subscription to the LiDAR topic (LaserScan messages)
        self.subscription = self.node.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)

    def laser_scan_callback(self, msg: LaserScan) -> None:
        self.laser_scan = msg

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill: Executing and processing LiDAR data.")
        # Wait for a LaserScan message if not already received (timeout after 3 seconds)
        timeout = time.time() + 3.0
        while self.laser_scan is None and time.time() < timeout:
            self.node.get_logger().info("Waiting for LiDAR sensor data...")
            time.sleep(0.1)

        if self.laser_scan is None:
            self.node.get_logger().warn("No LiDAR sensor data received within timeout.")
            return

        # Filter out invalid ranges (values outside the valid range_min and range_max)
        valid_ranges = [
            r for r in self.laser_scan.ranges
            if self.laser_scan.range_min < r < self.laser_scan.range_max
        ]
        if not valid_ranges:
            self.node.get_logger().warn("No valid LiDAR range readings received.")
            return

        # Compute the nearest distance
        self.nearest_distance = min(valid_ranges)
        index = self.laser_scan.ranges.index(self.nearest_distance)
        
        # Calculate the corresponding angle in radians then convert to degrees.
        angle_rad = self.laser_scan.angle_min + index * self.laser_scan.angle_increment
        self.nearest_angle = angle_rad * (180.0 / 3.14159265)

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(
            f"GetDistanceToNearestObjectSkill: Completed with nearest distance = {self.nearest_distance:.2f} m, angle = {self.nearest_angle:.2f}°"
        )

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("GetDistanceToNearestObjectSkill: Resetting, destroying LiDAR subscription.")
        self.node.destroy_subscription(self.subscription)
        self.laser_scan = None
        self.nearest_distance = None
        self.nearest_angle = None

# if __name__ == "__main__":
#     import rclpy
#     from rclpy.node import Node

#     class SkillNode(Node):
#         def __init__(self):
#             super().__init__("get_distance_to_nearest_object_skill_node")
#             # Instantiate the skill, which automatically registers it within the skill framework.
#             self.skill = GetDistanceToNearestObjectSkill()
#             # Create a ROS2 executor loop if needed – in practice, the skill framework will handle execution.
#             self.create_timer(1.0, self.timer_callback)

#         def timer_callback(self):
#             self.get_logger().info("Skill node running...")

#     rclpy.init()
#     node = SkillNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Skill node shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()