import json
from abc import ABC, abstractmethod
from typing import Optional, List
from prompt_handling.prompt_handling import PromptType, PromptHandler

class ControlEntity(ABC):
    def __init__(self, name: str, type: str, description: str = ""):
        self.name = name
        self.type = type
        self.description = description

    def get_name(self) -> str:
        return self.name
    
    def get_type(self) -> str:
        return self.type

    def get_description(self) -> str:
        return self.description
    
    def set_description(self, description: str):
        self.description = description

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
    def __init__(self, report: str, framework: str, resource_type: str, prompt_handler: PromptHandler):
        self.report = report
        self.framework = framework
        self.resource_type = resource_type
        self.prompt_handler = prompt_handler
        self.control_entities: list[ControlEntity] = []
        self.parse_report()
        self.generate_descriptions_for_rag()

    @abstractmethod
    def parse_report(self) -> None:
        pass

    def get_control_entities(self) -> list[ControlEntity]:
        return self.control_entities
    
    def generate_descriptions_for_rag(self): 
        """ Generate descriptions for RAG model. """
        for control_entity in (control_entity_without_description for control_entity_without_description in self.control_entities if control_entity_without_description.get_description() == ""):
            control_entity_description = self.prompt_handler.prompt(PromptType.RAG_PREPARATION, {"framework": self.framework, "resource_type": self.resource_type, "control_entity": control_entity.to_json(include=["name", "type"])})
            control_entity.set_description(control_entity_description.content)
        
        file_name = "ros2_descriptions.json"
        with open(file_name, "w") as json_file:
            json.dump([control_entity.to_dict(include=["name", "type", "description"]) for control_entity in self.control_entities], json_file, indent=4)

    def compare_control_entity_names(self, control_entities_json: List[dict]) -> List[ControlEntity]:
        entity_names = [entity["name"] for entity in control_entities_json]
        return [control_entity for control_entity in self.control_entities if control_entity.get_name() in entity_names]
    
    def get_framework(self) -> str:
        return self.framework
    
    def get_resource_type(self) -> str:
        return self.resource_type