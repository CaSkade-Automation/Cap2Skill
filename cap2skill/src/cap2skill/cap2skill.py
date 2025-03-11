import json
from typing import List
from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma
from langchain.schema import Document
from langchain_core.messages import BaseMessage
from report_handling.control_report_handling import ControlReportHandling, ControlEntity
from prompt_handling.prompt_handling import PromptHandler, PromptType

class Cap2Skill:
    """ Class for processing prompts and communicating with LLM. """

    def __init__(self, language: str, capability: str, context_organizer: ControlReportHandling, prompt_handler: PromptHandler):
        self.embeddings = OpenAIEmbeddings(model="text-embedding-3-large")
        self.context_organizer = context_organizer
        self.language = language
        self.capability = capability
        self.prompt_handler = prompt_handler
        self.vector_store = None

    def generate_chunks_from_context(self) -> list[Document]:
        chunks = []

        for control_entity in self.context_organizer.get_control_entities():
            if self.context_organizer.get_framework() == "ROS2":
                chunks.append(control_entity.to_json(include=["name", "type", "description"], interfaces = False))

        chunk_documents = [Document(page_content=text) for text in chunks]
        return chunk_documents 
    
    def set_vector_store(self, chunk_documents: List[Document]):
        """ Set the vector store for the chat model. """
        self.vector_store = Chroma.from_documents(chunk_documents, self.embeddings, collection_metadata={"hnsw:space": "cosine"})
    
    def retrieve_relevant_context(self) -> List[ControlEntity]: 
        """ Retrieve relevant context for the given capability ontology. """
        query = f"Find relevant {self.context_organizer.get_framework()} interfaces for the following capability: {self.capability}"
        #TODO: optimize the number of retrieved contexts
        retrieved_contexts = self.vector_store.similarity_search(query, 4) 
        retrieved_control_entities = self.context_organizer.compare_control_entity_names([json.loads(retrieved_context.page_content) for retrieved_context in retrieved_contexts])

        return retrieved_control_entities

    def generate_skill_code(self) -> BaseMessage:
        """ Generate code from an OWL capability ontology. """

        chunk_documents = self.generate_chunks_from_context()
        self.set_vector_store(chunk_documents)
        retrieved_control_entities = self.retrieve_relevant_context()
        skill = self.prompt_handler.prompt(PromptType.SKILL_GENERATION, {"language": self.language, "framework": self.context_organizer.get_framework(), "resource_type": self.context_organizer.get_resource_type(), "capability": self.capability, "control_entities": [retrieved_control_entity.to_dict(include=["name", "type", "description"]) for retrieved_control_entity in retrieved_control_entities]})
        return skill
    
if __name__ == "__main__":
    import os 
    from prompt_handling.prompt_handling import load_file
    from report_handling.ros2_handling import ROS2ReportHandling

    context_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "../docs", "ros2_system_report_with_descriptions.json") # important: for testing only use of system reports with descriptions otherwise prompting is done for descriptions
    ontology_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "../../capability-models", "set-velocity.ttl")
    context = load_file(context_path)
    ontology = load_file(ontology_path)

    prompt_handler = PromptHandler()

    control_entities_handler = ROS2ReportHandling(context, "ROS2", "Mobile Robot", prompt_handler)

    cap2skill = Cap2Skill("Python", ontology, control_entities_handler, prompt_handler)
    chunks = cap2skill.generate_chunks_from_context()
    cap2skill.set_vector_store(chunks)
    retrieved_control_entities = cap2skill.retrieve_relevant_context()
    
    file_name = "ros2_retrieved_context.txt"
    with open(file_name, "w") as file:
        json.dump([retrieved_control_entity.to_dict(include=["name", "type", "description"]) for retrieved_control_entity in retrieved_control_entities], file, indent=4)