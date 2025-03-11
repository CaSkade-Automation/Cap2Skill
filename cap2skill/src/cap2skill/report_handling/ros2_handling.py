import json
from report_handling.control_report_handling import ControlReportHandling, ControlEntity
from prompt_handling.prompt_handling import PromptHandler
from typing import Optional, List

class ROS2Interface: 
    """ Class for processing ROS2 interfaces (messages, services and actions). """
    name: str
    details: str

    def __init__(self, name: str, details: str):
        self.name = name
        self.details = details

class ROS2ControlEntity(ControlEntity):
    """ Class for processing ROS2 control entities. """

    def __init__(self, name: str, communication_type: str, interfaces: list[ROS2Interface], description: str = ""):
        super().__init__(name, communication_type, description)
        self.interfaces = interfaces

    def to_dict(self, include: Optional[List[str]] = None, interfaces: bool = True) -> dict:
        control_entity_dict = super().to_dict(include)
        if interfaces:
            control_entity_dict["interfaces"] = [interface.__dict__ for interface in self.interfaces]
        return control_entity_dict
    
    def to_json(self, include: Optional[List[str]] = None, interfaces: bool = True) -> str:
        return json.dumps(self.to_dict(include, interfaces), indent=4)


class ROS2ReportHandling(ControlReportHandling): 
    def __init__(self, ros2_report: str, framework: str, resource_type: str, prompt_handler: PromptHandler):
        super().__init__(ros2_report, framework, resource_type, prompt_handler)

    def parse_report(self) -> None:
        """ Parse ROS2 system report and return relevant parts. """
        ros2_report_json = json.loads(self.report)

        if "ros2_control_entities" not in ros2_report_json:
            print("Invalid report format: Missing 'ros2_control_entities' section.")
            return
        
        for entities in ros2_report_json["ros2_control_entities"].values():
            for entity in entities:
                interfaces = [ROS2Interface(interface["name"], interface["details"]) for interface in entity["interfaces"]]
                description = entity["description"] if "description" in entity else ""
                ros2_entity = ROS2ControlEntity(entity["name"], entity["type"], interfaces, description)
                self.control_entities.append(ros2_entity)

        print(f"Loaded {len(self.control_entities)} ROS2 entities from report.")

    def get_control_entities(self) -> list[ROS2ControlEntity]:
        return self.control_entities