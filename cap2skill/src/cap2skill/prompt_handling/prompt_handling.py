import os
import getpass
from enum import Enum
from dotenv import load_dotenv
from langchain.chat_models import init_chat_model
from langchain.prompts import ChatPromptTemplate
from langchain_core.messages import BaseMessage
from typing import Dict

def load_file(file_path): 
        with open(file_path, "r") as f:
            return f.read()

BASE_DIR = os.path.dirname(os.path.dirname(__file__))

class PromptType(Enum):
    RAG_PREPARATION = "RAG_Preparation"
    SKILL_GENERATION = "Skill_Generation"

def get_prompt_type(file_name: str) -> PromptType:
    if "rag_prep" in file_name:
        return PromptType.RAG_PREPARATION
    elif "skill_generation" in file_name:
        return PromptType.SKILL_GENERATION
    return None

class PromptHandler():
    def __init__(self):
        self.prompt_templates = self.generate_prompt_templates()
        
        load_dotenv()
        # TODO: Possibility to use other LLMs 
        if not os.environ.get("OPENAI_API_KEY"):
            os.environ["OPENAI_API_KEY"] = getpass.getpass("Enter OpenAI API key: ")

        self.llm = init_chat_model('gpt-4', model_provider='openai', temperature=0)


    def generate_prompt_templates(self) -> Dict[PromptType, ChatPromptTemplate]:
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
        
        return prompt_template
        
    def prompt(self, prompt_type: PromptType, variables: dict[str, str]) -> BaseMessage:
        prompt_template = self.prompt_templates[prompt_type]
        prompt = prompt_template.invoke(variables)
        answer = self.llm.invoke(prompt)
        return answer