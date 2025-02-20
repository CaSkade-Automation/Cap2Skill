import os
import getpass
from langchain_openai import OpenAIEmbeddings
from langchain.chat_models import init_chat_model
from langchain_community.vectorstores import Chroma
from langchain.schema import Document
from langchain.prompts import ChatPromptTemplate
from dotenv import load_dotenv

def load_file(file_path): 
    with open(file_path, "r") as f:
        return f.read()

load_dotenv()
if not os.environ.get("OPENAI_API_KEY"):
    os.environ["OPENAI_API_KEY"] = getpass.getpass("Enter OpenAI API key: ")

# 0. Path to Markdown file
base_dir = os.path.dirname(os.path.dirname(__file__))
doc_path = os.path.join(base_dir, "../docs", "ros2_system_report.md")

# 1. Load document
document_text = load_file(doc_path)

# 2. Split document into manageable chunks
chunks = document_text.split("**/")
print(f"split ros2 information into {len(chunks)} chunks")

# 3. Create embeddings for the document chunks
llm = init_chat_model('gpt-4', model_provider='openai')
embeddings = OpenAIEmbeddings(model="text-embedding-3-large")

# If chunks is a list of strings, convert them to document objects
documents = [Document(page_content=text) for text in chunks]

# 4. Create vectorstore (Chroma) from the documents
vector_store = Chroma.from_documents(documents, embeddings)

# Prompting
language = "Python"
framework = "ROS2"
resource_type = "mobile robots"

system_prompt_template_path = os.path.join(base_dir, "../prompt-templates", "system_prompt.txt")
system_prompt_template = load_file(system_prompt_template_path)

user_prompt_template_path = os.path.join(base_dir, "../prompt-templates", "user_prompt.txt")
user_prompt_template = load_file(user_prompt_template_path)

prompt_template = ChatPromptTemplate.from_messages(
    [("system", system_prompt_template), ('user', user_prompt_template)]
)

ontology_path = os.path.join(base_dir, "../../capability-models", "set-velocity.ttl")
ontology = load_file(ontology_path)


query = f"Generate {language} {framework} code from an OWL ontology in Turtle syntax for a {resource_type}.  {ontology}"
retrieved_docs = vector_store.similarity_search(query)
docs_content = "\n\n".join(doc.page_content for doc in retrieved_docs)
prompt = prompt_template.invoke({"language": language, "framework": framework, "resource_type": resource_type, "control_options": docs_content, "capability": ontology})
answer = llm.invoke(prompt)

print(answer)
