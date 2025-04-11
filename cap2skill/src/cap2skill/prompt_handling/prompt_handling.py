import os
import getpass
import logging
from enum import Enum
from dotenv import load_dotenv
from langchain.chat_models import init_chat_model
from langchain.prompts import ChatPromptTemplate
from langchain_core.messages import BaseMessage
from langchain_openai import ChatOpenAI
from typing import Dict

def load_file(file_path): 
        with open(file_path, "r") as f:
            return f.read()

BASE_DIR = os.path.dirname(os.path.dirname(__file__))

class PromptType(Enum):
    RAG_PREPARATION = "RAG_Preparation"
    SKILL_GENERATION = "Skill_Generation"

def get_prompt_type(file_name: str) -> PromptType:
    """
    Determine the type of prompt based on the provided file name.

    Args:
        file_name (str): The name of the file to analyze.

    Returns:
        PromptType: The type of prompt inferred from the file name. Returns
        `PromptType.RAG_PREPARATION` if "rag_prep" is in the file name,
        `PromptType.SKILL_GENERATION` if "skill_generation" is in the file name,
        or `None` if no matching type is found.
    """
    if "rag_prep" in file_name:
        return PromptType.RAG_PREPARATION
    elif "skill_generation" in file_name:
        return PromptType.SKILL_GENERATION
    return None

class PromptHandler():
    """
    A class to handle prompt generation and interaction with a language model (LLM).
    Attributes:
        logger (logging.Logger): Logger instance for logging messages.
        prompt_templates (Dict[PromptType, ChatPromptTemplate]): Dictionary of prompt templates categorized by prompt type.
        llm: Instance of the initialized language model.
    Methods:
        __init__():
            Initializes the PromptHandler, sets up logging, loads environment variables, and initializes the LLM.
        generate_prompt_templates() -> Dict[PromptType, ChatPromptTemplate]:
            Generates and loads prompt templates from the specified directory.
        prompt(prompt_type: PromptType, variables: dict[str, str]) -> BaseMessage:
            Generates a prompt based on the specified type and variables, sends it to the LLM, and returns the response.
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__.split(".")[-1])
        self.prompt_templates = self.generate_prompt_templates()
        
        load_dotenv()
        # TODO: Possibility to use other LLMs 
        if not os.environ.get("OPENAI_API_KEY"):
            os.environ["OPENAI_API_KEY"] = getpass.getpass("Enter OpenAI API key: ")

        self.logger.info("Loaded OpenAI API key.")
        self.llm = init_chat_model('gpt-4o', model_provider='openai', temperature=0, top_p=1, frequency_penalty=0, presence_penalty=0)
        self.logger.info("Loaded LLM model: gpt-4o")

    def generate_prompt_templates(self) -> Dict[PromptType, ChatPromptTemplate]:
        """
        Generates a dictionary of prompt templates by loading and combining system 
        and user prompt files from the specified directory.
        This method scans a directory for prompt files, identifies matching system 
        and user prompt pairs, and creates `ChatPromptTemplate` objects for each 
        valid prompt type. The resulting templates are stored in a dictionary 
        where the keys are `PromptType` values and the values are the corresponding 
        `ChatPromptTemplate` objects.
        Returns:
            Dict[PromptType, ChatPromptTemplate]: A dictionary mapping prompt types 
            to their respective chat prompt templates.
        Raises:
            FileNotFoundError: If the prompt directory or any required prompt file 
            is missing.
            ValueError: If a prompt type cannot be determined for a matching pair 
            of system and user prompts.
        """
        self.logger.info("Preparing prompt templates.")
        prompt_dir = os.path.join(BASE_DIR, "../../prompt-templates")
        prompt_files = os.listdir(prompt_dir)

        system_prompts = {f.replace("system_", ""): f for f in prompt_files if f.startswith("system_")}
        user_prompts = {f.replace("user_", ""): f for f in prompt_files if f.startswith("user_")}

        prompt_templates: Dict[PromptType, ChatPromptTemplate] = {}

        for key in system_prompts.keys() & user_prompts.keys():
            prompt_type = get_prompt_type(key)
            if not prompt_type:
                continue

            system_prompt_path = os.path.join(prompt_dir, system_prompts[key])
            user_prompt_path = os.path.join(prompt_dir, user_prompts[key])

            system_prompt_template = load_file(system_prompt_path)
            user_prompt_template = load_file(user_prompt_path)

            prompt_template = ChatPromptTemplate.from_messages(
                [("system", system_prompt_template), ('user', user_prompt_template)]
            )
            prompt_templates[prompt_type] = prompt_template
            self.logger.info(f"Loaded prompt template for {prompt_type}")
        
        return prompt_templates
        
    def prompt(self, prompt_type: PromptType, variables: dict[str, str]) -> BaseMessage:
        """
        Generates a response based on the specified prompt type and variables.

        Args:
            prompt_type (PromptType): The type of prompt to be used.
            variables (dict[str, str]): A dictionary of variables to be substituted 
                into the prompt template.

        Returns:
            BaseMessage: The response generated by the language model (LLM) 
            after invoking the prompt.
        """
        self.logger.info(f"Prompting for {prompt_type}")
        prompt_template = self.prompt_templates[prompt_type]
        prompt = prompt_template.invoke(variables)
        answer = self.llm.invoke(prompt)
        return answer