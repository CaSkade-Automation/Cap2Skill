import os
import getpass
from langchain_openai import OpenAIEmbeddings
from langchain.chat_models import init_chat_model
from langchain_community.vectorstores import Chroma
from langchain.schema import Document
from langchain.prompts import ChatPromptTemplate
from langchain_core.prompt_values import PromptValue
from langchain_core.messages import BaseMessage
from dotenv import load_dotenv

def load_file(file_path): 
        with open(file_path, "r") as f:
            return f.read()

BASE_DIR = os.path.dirname(os.path.dirname(__file__))

def load_prompt_template() -> ChatPromptTemplate:
    system_prompt_template_path = os.path.join(BASE_DIR, "../prompt-templates", "system_prompt.txt")
    system_prompt_template = load_file(system_prompt_template_path)

    user_prompt_template_path = os.path.join(BASE_DIR, "../prompt-templates", "user_prompt.txt")
    user_prompt_template = load_file(user_prompt_template_path)

    prompt_template = ChatPromptTemplate.from_messages(
        [("system", system_prompt_template), ('user', user_prompt_template)]
    )
    return prompt_template

load_dotenv()
if not os.environ.get("OPENAI_API_KEY"):
    os.environ["OPENAI_API_KEY"] = getpass.getpass("Enter OpenAI API key: ")


class Cap2Skill:
    """ Class for processing prompts and communicating with LLM. """

    def __init__(self, programming_language: str, framework: str, resource_type: str, context: str, ontology: str):
        self.llm = init_chat_model('gpt-4', model_provider='openai')
        self.embeddings = OpenAIEmbeddings(model="text-embedding-3-large")
        self.programming_language = programming_language
        self.framework = framework
        self.resource_type = resource_type
        self.ontology = ontology
        self.context = context
        self.vector_store = None
        self.prompt_template = load_prompt_template()

    def generate_chunks_from_context(self) -> list[Document]:
        """ Split context into manageable chunks. """
        chunks = self.context.split("**/")
        print(f"split ros2 information into {len(chunks)} chunks")
        
        # If chunks is a list of strings, convert them to document objects
        chunk_documents = [Document(page_content=text) for text in chunks]
        return chunk_documents 
    
    def set_vector_store(self, chunk_documents: list[Document]):
        """ Set the vector store for the chat model. """
        self.vector_store = Chroma.from_documents(chunk_documents, self.embeddings)
        
    
    def retrieve_relevant_context(self) -> str: 
        """ Retrieve relevant context for the given capability ontology. """
        query = f"Generate {self.programming_language} {self.framework} code from an OWL ontology in Turtle syntax for a {self.resource_type}.  {self.ontology}"
        retrieved_contexts = self.vector_store.similarity_search(query)
        retrieved_context_string = "\n\n".join(retrieved_context.page_content for retrieved_context in retrieved_contexts)
        return retrieved_context_string

    def generate_prompt(self, retrieved_context_string: str) -> PromptValue:
        prompt = self.prompt_template.invoke({"language": self.programming_language, "framework": self.framework, "resource_type": self.resource_type, "control_options": retrieved_context_string, "capability": self.ontology})
        return prompt

    def generate_skill_code(self) -> BaseMessage:
        """ Generate code from an OWL capability ontology. """

        chunk_documents = self.generate_chunks_from_context()
        self.set_vector_store(chunk_documents)
        retrieved_context_string = self.retrieve_relevant_context()
        prompt = self.generate_prompt(retrieved_context_string)

        answer = self.llm.invoke(prompt)
        return answer

# for testing purposes
if __name__ == "__main__":
    context_path = os.path.join(BASE_DIR, "../docs", "ros2_system_report.md")
    ontology_path = os.path.join(BASE_DIR, "../../capability-models", "set-velocity.ttl")
    context = load_file(context_path)
    ontology = load_file(ontology_path)
    cap2skill = Cap2Skill("Python", "ROS2", "Mobile Robots", context, ontology)

    chunks = cap2skill.generate_chunks_from_context()
    cap2skill.set_vector_store(chunks)
    retrieved_context = cap2skill.retrieve_relevant_context()
    
    file_name = "ros2_retrieved_context.txt"
    with open(file_name, "w") as file:
        file.write(retrieved_context)