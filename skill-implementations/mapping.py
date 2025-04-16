import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
import math
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan

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
        self.subscription = None
        # Service client to request the generated map
        self.get_map_client = None
        # Store the generated map as a YAML-like string (summary for this skill)
        self.generated_map = ""
        # Mapping area dimensions as provided by the user (in meters)
        self.area_length_x = 0.0  
        self.area_length_y = 0.0
        self.obstacle_detected = False 
        self.received_scan = False  

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
        self.subscription = self.node.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        # Create client for the GetMap service provided by slam_toolbox/dynamic_map.
        self.get_map_client = self.node.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("MappingSkill: Waiting for /slam_toolbox/dynamic_map service...")
        self.node.get_logger().info("MappingSkill: /slam_toolbox/dynamic_map service is now available.")

    def _scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_ranges = len(ranges)

        def angle_to_index(angle_deg):
            angle_rad = angle_deg * 3.14159 / 180.0
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            return max(0, min(index, num_ranges - 1))

        # Richtungsbereiche in Grad:
        front_range = range(angle_to_index(-10), angle_to_index(10))
        left_range = range(angle_to_index(80), angle_to_index(100))
        right_range = range(angle_to_index(-100), angle_to_index(-80))
        back_range = range(angle_to_index(170), angle_to_index(180))

        def min_valid_distance(index_range):
            valid = [ranges[i] for i in index_range if not (math.isnan(ranges[i]) or math.isinf(ranges[i]))]
            return min(valid) if valid else float('inf')

        import math  # falls oben nicht importiert
        self.min_front = min_valid_distance(front_range)
        self.min_left = min_valid_distance(left_range)
        self.min_right = min_valid_distance(right_range)
        self.min_back = min_valid_distance(back_range)

        self.obstacle_detected = self.min_front < 0.5  # Schwellenwert für Hindernis vorne

        # (Optional) Debug-Ausgabe
        self.node.get_logger().info(
            f"Scan: Front={self.min_front:.2f}m, Left={self.min_left:.2f}m, "
            f"Right={self.min_right:.2f}m, Back={self.min_back:.2f}m"
        )

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("MappingSkill: Executing – starting mapping routine using a snake pattern.")
        # Simulation parameters for movement
        forward_speed = 0.2    # m/s (linear speed)
        turn_speed = 0.5       # rad/s (angular speed)
        # Calculate time required to traverse a row (distance = speed * time)
        abdeckfläche = self.area_length_x * self.area_length_y
        basisdauer = abdeckfläche / (forward_speed * 0.8)
        zeitfaktor = 2.0  # konservativ, kann je nach Umgebung angepasst werden
        max_duration = basisdauer * zeitfaktor
        twist = Twist()
        start_time = time.time()

        while time.time() - start_time < max_duration:
            if not self.obstacle_detected:
                # Kein Hindernis: fahre geradeaus
                twist.linear.x = forward_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                # Hindernis erkannt: stoppen und Richtung wechseln
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)

                # Richtung mit größtem Freiraum bestimmen
                direction, free_distance = self._find_best_direction()
                if direction is None:
                    self.node.get_logger().error("MappingSkill: Kein Ausweg gefunden – Stoppe Roboter.")
                    break

                self.node.get_logger().info(f"MappingSkill: Drehe in Richtung {direction}° (freier Bereich: {free_distance:.2f} m).")
                angle_rad = direction * 3.14159 / 180.0
                duration = abs(angle_rad) / turn_speed

                twist.angular.z = turn_speed if angle_rad > 0 else -turn_speed
                self._rotate_for_duration(twist, duration)

                # Danach wieder versuchen, vorwärts zu fahren
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)

            time.sleep(0.1)

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

    def _find_best_direction(self):
        threshold = 0.6  # Minimaler freier Abstand in Metern

        def is_valid(d): return d != float('inf') and not math.isnan(d) and d > threshold

        directions = []

        if is_valid(self.min_left):
            directions.append((90, self.min_left))
        if is_valid(self.min_right):
            directions.append((-90, self.min_right))
        if is_valid(self.min_back):
            directions.append((180, self.min_back))

        # Wenn keine Richtung verfügbar ist
        if not directions:
            return None, 0.0

        # Beste Richtung wählen (größter Abstand)
        best = max(directions, key=lambda x: x[1])
        return best

    def _rotate_for_duration(self, twist_msg, duration):
        t_end = time.time() + duration
        while time.time() < t_end:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        time.sleep(0.5)

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