import json
import logging
from abc import ABC, abstractmethod
from typing import Optional, List
from prompt_handling.prompt_handling import PromptType, PromptHandler

class ControlEntity(ABC):
    """
    ControlEntity is an abstract base class (ABC) that represents a control entity with a name, type, and optional description.
    It provides methods to retrieve and modify its attributes, as well as to serialize the entity into dictionary or JSON formats.
    Attributes:
        name (str): The name of the control entity.
        type (str): The type of the control entity.
        description (str): An optional description of the control entity.
        logger (logging.Logger): A logger instance for the class.
    Methods:
        get_name() -> str:
            Returns the name of the control entity.
        get_type() -> str:
            Returns the type of the control entity.
        get_description() -> str:
            Returns the description of the control entity.
        set_description(description: str):
            Sets the description of the control entity.
        to_dict(include: Optional[List[str]] = None) -> dict:
            Converts the control entity to a dictionary. If `include` is provided, only the specified keys are included.
        to_json(include: Optional[List[str]] = None) -> str:
            Converts the control entity to a JSON string. If `include` is provided, only the specified keys are included.
    """
    def __init__(self, name: str, type: str, description: str = ""):
        self.name = name
        self.type = type
        self.description = description
        self.logger = logging.getLogger(__name__.split(".")[-1])

    def get_name(self) -> str:
        return self.name
    
    def get_type(self) -> str:
        return self.type

    def get_description(self) -> str:
        return self.description
    
    def set_description(self, description: str):
        self.description = description
        self.logger.info(f"Description set for {self.name}")

    def to_dict(self, include: Optional[List[str]] = None) -> dict:
        control_entity_dict = {
            "name": self.name,
            "type": self.type,
            "description": self.description
        }

        if include:
            return {key: control_entity_dict[key] for key in include}
        return control_entity_dict
    
    def to_json(self, include: Optional[List[str]] = None) -> str:
        return json.dumps(self.to_dict(include), indent=4)

class ControlReportHandling(ABC): 
    """
    ControlReportHandling is an abstract base class (ABC) that provides a framework for handling control reports. 
    It includes methods for generating reports, parol entities, and generating descriptionsing reports, managing contrs 
    for use in a Retrieval-Augmented Generation (RAG) model.
    Attributes:
        framework (str): The framework associated with the control report (e.g., ROS 2).
        resource_type (str): The type of resource associated with the control report.
        prompt_handler (PromptHandler): An instance of PromptHandler used for interacting with an LLM.
        report (str): The control report content. If not provided, it will be generated.
        control_entities (list[ControlEntity]): A list of ControlEntity objects parsed from the report.
        logger (logging.Logger): Logger instance for logging messages.
    Methods:
        generate_report() -> str:
            Abstract method to generate the control report. Must be implemented by subclasses.
        parse_report() -> None:
            Abstract method to parse the control report. Must be implemented by subclasses.
        get_control_entities() -> list[ControlEntity]:
            Returns the list of control entities parsed from the report.
        generate_descriptions_for_rag() -> None:
            Generates descriptions for control entities that lack descriptions for use in RAG model.
        compare_control_entity_names(control_entities_json: List[dict]) -> List[ControlEntity]:
            Compares the names of control entities in the provided JSON with the existing control entities 
            and returns a list of matching ControlEntity objects.
        get_framework() -> str:
            Returns the framework associated with the control report.
        get_resource_type() -> str:
            Returns the resource type associated with the control report.
        to_dict(include: Optional[List[str]] = None) -> dict:
            Converts the control report and its entities into a dictionary representation. 
            Optionally includes only specified keys.
        to_json(include: Optional[List[str]] = None) -> str:
            Converts the control report and its entities into a JSON string representation. 
            Optionally includes only specified keys.
    """
    def __init__(self, framework: str, resource_type: str, prompt_handler: PromptHandler, report: str = ""):
        self.framework = framework
        self.resource_type = resource_type
        self.prompt_handler = prompt_handler
        self.control_entities: list[ControlEntity] = []
        self.logger = logging.getLogger(__name__.split(".")[-1])
        if not report:
            self.report = self.generate_report()
        else:
            self.report = report
        self.parse_report()
        self.generate_descriptions_for_rag()

    @abstractmethod
    def generate_report(self) -> str:
        pass

    @abstractmethod
    def parse_report(self) -> None:
        pass

    def get_control_entities(self) -> list[ControlEntity]:
        return self.control_entities
    
    def generate_descriptions_for_rag(self): 
        """
        Generate descriptions for control entities for use in RAG (Retrieval-Augmented Generation) model.

        This method iterates through all control entities that do not have a description and generates
        a description for each using the specified prompt handler. The generated description is then
        set for the respective control entity.

        Logging:
            - Logs the start of the description generation process.
            - Logs the name of each control entity for which a description is being generated.

        Uses:
            - The `prompt_handler` to generate descriptions based on the RAG_PREPARATION prompt type.
            - The `framework` and `resource_type` attributes to provide context for the prompt.
            - The `to_json` method of the control entity to serialize relevant fields for the prompt.

        Raises:
            - Any exceptions raised by the `prompt_handler` or `set_description` methods.

        """
        self.logger.info("Generating descriptions for RAG model.")
        for control_entity in (control_entity_without_description for control_entity_without_description in self.control_entities if control_entity_without_description.get_description() == ""):
            self.logger.info(f"Generating description for {control_entity.get_name()}.")
            control_entity_description = self.prompt_handler.prompt(PromptType.RAG_PREPARATION, {"framework": self.framework, "resource_type": self.resource_type, "control_entity": control_entity.to_json(include=["name", "type"])})
            control_entity.set_description(control_entity_description.content)

    def compare_control_entity_names(self, control_entities_json: List[dict]) -> List[ControlEntity]:
        entity_names = [entity["name"] for entity in control_entities_json]
        return [control_entity for control_entity in self.control_entities if control_entity.get_name() in entity_names]
    
    def get_framework(self) -> str:
        return self.framework
    
    def get_resource_type(self) -> str:
        return self.resource_type
    
    def to_dict(self, include: Optional[List[str]] = None) -> dict:
        control_report_dict = {
            "control_entities": [control_entity.to_dict(include) for control_entity in self.control_entities]
        }

        if include:
            return {key: control_report_dict[key] for key in include}
        return control_report_dict
    
    def to_json(self, include: Optional[List[str]] = None) -> str:
        return json.dumps(self.to_dict(include), indent=4)