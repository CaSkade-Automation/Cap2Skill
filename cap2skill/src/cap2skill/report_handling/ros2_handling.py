import json
from report_handling.control_report_handling import ControlReportHandling, ControlEntity
from report_handling.ros2_report_generation import generate_report
from prompt_handling.prompt_handling import PromptHandler
from typing import Optional, List, Union

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
    def __init__(self, framework: str, resource_type: str, prompt_handler: PromptHandler, ros2_report: Union[str, dict] = ""):
        super().__init__(framework, resource_type, prompt_handler, ros2_report)

    def generate_report(self) -> str:
        """ Generate ROS2 system report. """
        self.logger.info("Start generating ROS2 system report.")
        return generate_report()

    def parse_report(self) -> None:
        """
        Parse a ROS2 system report and extract relevant information.
        This method processes a JSON-formatted ROS2 system report, extracting
        control entities and their associated interfaces. The extracted entities
        are stored in the `control_entities` attribute.
        Steps:
        1. Logs the start of the parsing process.
        2. Validates the presence of the "ros2_control_entities" section in the report.
        3. Iterates through the entities, extracting their name, type, interfaces,
            and optional description.
        4. Creates `ROS2ControlEntity` objects for each entity and appends them
            to the `control_entities` list.
        5. Logs the total number of entities loaded.
        Returns:
            None
        Logs:
            - Info: Start and end of the parsing process, and details of each entity being parsed.
            - Error: If the report format is invalid or missing required sections.
        Raises:
            None
        """
        # TODO: make function flexible for different report formats
        self.logger.info("Start parsing ROS2 system report.")

        if isinstance(self.report, str):
            ros2_report_json = json.loads(self.report)
        elif isinstance(self.report, dict):
            ros2_report_json = self.report
        else:
            self.logger.error("Report must be a JSON string or dict.")

        if "ros2_control_entities" not in ros2_report_json:
            self.logger.error("Invalid report format: Missing 'ros2_control_entities' section.")
            return
        
        for entity in ros2_report_json["ros2_control_entities"]:
            self.logger.info(f"Parsing entity: {entity['name']}")
            interfaces = [ROS2Interface(interface["name"], interface["details"]) for interface in entity["interfaces"]]
            description = entity["description"] if "description" in entity else ""
            ros2_entity = ROS2ControlEntity(entity["name"], entity["type"], interfaces, description)
            self.control_entities.append(ros2_entity)

        self.logger.info(f"Loaded {len(self.control_entities)} ROS2 entities from report.")

    def get_control_entities(self) -> list[ROS2ControlEntity]:
        return self.control_entities