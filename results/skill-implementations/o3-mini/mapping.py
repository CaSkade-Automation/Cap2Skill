import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface

from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap

@skill(
    skill_interface = SkillInterface.REST,
    skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/mapping/mapping",
    module_iri= "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700",
    capability_iri = "http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/mapping/mapping",
    description = "Map an area of a desired length x and y based on the mobile robot's position by moving the mobile robot in that area."
)
class MappingSkill(ROS2Skill):

    def __init__(self):
        super().__init__("mapping")
        # Publisher to control robot velocity (simulate mapping movement)
        self.cmd_vel_pub = None
        # Service client to request the generated map
        self.get_map_client = None
        # Store the generated map as a YAML-like string (summary for this skill)
        self.generated_map = ""
        # Mapping area dimensions as provided by the user (in meters)
        self.area_length_x = 0.0  
        self.area_length_y = 0.0  

    @skill_parameter(is_required=True, name="area_length_x", 
                     description="Desired mapping area length in the x direction (meters).")
    def get_area_length_x(self) -> float:
        return self.area_length_x

    @skill_parameter(is_required=True, name="area_length_y", 
                     description="Desired mapping area length in the y direction (meters).")
    def get_area_length_y(self) -> float:
        return self.area_length_y

    @skill_output(is_required=True, name="generated_map", 
                  description="The generated map as a summary string containing key parameters.")
    def get_generated_map(self) -> str:
        return self.generated_map

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MappingSkill: Starting – initializing publishers and service clients.")
        # Create publisher for cmd_vel to command robot movements during mapping.
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        # Create client for the GetMap service provided by slam_toolbox/dynamic_map.
        self.get_map_client = self.node.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("MappingSkill: Waiting for /slam_toolbox/dynamic_map service...")
        self.node.get_logger().info("MappingSkill: /slam_toolbox/dynamic_map service is now available.")

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("MappingSkill: Executing – starting mapping routine using a snake pattern.")
        # Simulation parameters for movement
        forward_speed = 0.2    # m/s (linear speed)
        turn_speed = 0.5       # rad/s (angular speed)
        # Calculate time required to traverse a row (distance = speed * time)
        row_duration = self.area_length_x / forward_speed if forward_speed > 0 else 0
        # Define row spacing (meters) and compute number of rows covering the area in y direction.
        row_spacing = 0.5      # meters between rows
        num_rows = int(self.area_length_y / row_spacing) if row_spacing > 0 else 1
        self.node.get_logger().info(f"MappingSkill: Planning snake pattern with {num_rows} rows.")

        # Execute a snake (zig-zag) pattern across the area.
        for row in range(num_rows):
            self.node.get_logger().info(f"MappingSkill: Traversing row {row+1} of {num_rows}.")
            # Move forward along the row.
            t_end = time.time() + row_duration
            twist_msg = Twist()
            twist_msg.linear.x = forward_speed
            while time.time() < t_end:
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.1)
            # Stop at the end of the row.
            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.5)

            # Simulate obstacle handling in the second row if applicable.
            if row == 1:
                self.node.get_logger().info("MappingSkill: Obstacle detected! Executing turn maneuver.")
                twist_turn = Twist()
                twist_turn.angular.z = turn_speed
                t_turn = time.time() + 2.0  # turn for 2 seconds to avoid obstacle
                while time.time() < t_turn:
                    self.cmd_vel_pub.publish(twist_turn)
                    time.sleep(0.1)
                # Stop rotation.
                twist_turn.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_turn)
                time.sleep(0.5)

            # If not the last row, turn 90° to prepare for the next pass.
            if row < num_rows - 1:
                self.node.get_logger().info("MappingSkill: Turning 90° to start the next row.")
                twist_turn = Twist()
                twist_turn.angular.z = turn_speed
                t_turn = time.time() + (1.57 / turn_speed)  # approximate time to turn 90°
                while time.time() < t_turn:
                    self.cmd_vel_pub.publish(twist_turn)
                    time.sleep(0.1)
                twist_turn.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_turn)
                time.sleep(0.5)

        # After finishing the movement pattern, stop the robot.
        twist_stop = Twist()
        self.cmd_vel_pub.publish(twist_stop)
        self.node.get_logger().info("MappingSkill: Movement pattern completed. Requesting generated map from SLAM service.")

        # Request the generated map via the GetMap service.
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        try:
            response = future.result()
            map_info = response.map.info
            self.generated_map = (
                f"Map Summary: width={map_info.width}, height={map_info.height}, "
                f"resolution={map_info.resolution}, frame_id={response.map.header.frame_id}"
            )
            self.node.get_logger().info("MappingSkill: Map received successfully.")
        except Exception as e:
            self.node.get_logger().error(f"MappingSkill: Failed to retrieve map: {e}")
            self.generated_map = "Map retrieval failed."

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MappingSkill: Stopping – sending zero velocity command to stop the robot.")
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    @completing
    def completing(self) -> None:
        self.node.get_logger().info(f"MappingSkill: Completing – {self.generated_map}")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MappingSkill: Resetting – cleaning up publishers and service clients.")
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.destroy()
        if self.get_map_client is not None:
            self.get_map_client.destroy()