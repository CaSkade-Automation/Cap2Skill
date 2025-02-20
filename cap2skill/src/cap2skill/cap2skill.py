import os
import getpass
from langchain_openai import OpenAIEmbeddings
from langchain.chat_models import init_chat_model
from langchain_community.vectorstores import Chroma
from langchain.chains import RetrievalQA
from langchain.schema import Document
from langchain.prompts import ChatPromptTemplate
from langchain import hub

if not os.environ.get("OPENAI_API_KEY"):
    os.environ["OPENAI_API_KEY"] = getpass.getpass("Enter OpenAI API key: ")

# 0. Path to Markdown file
base_dir = os.path.dirname(os.path.dirname(__file__))
doc_path = os.path.join(base_dir, "../docs", "ros2_system_report.md")

# 1. Load document
with open(doc_path, "r") as f:
    document_text = f.read()

# 2. Split document into manageable chunks
chunks = document_text.split("**/")
print(f"split ros2 information into {len(chunks)} chunks")

# 3. Create embeddings for the document chunks
llm = init_chat_model('gpt-4', model_provider='openai')
embeddings = OpenAIEmbeddings(model="text-embedding-3-large")

# Falls chunks eine Liste von Strings ist, konvertiere sie in Document-Objekte
documents = [Document(page_content=text) for text in chunks]

# 4. Create vectorstore (Chroma) from the documents
vector_store = Chroma.from_documents(documents, embeddings)

# Prompting
language = "Python"
framework = "ROS2"
resource_type = "mobile robots"
system_template = "You are a programming expert in {language} with {framework} specializing in the automatic generation of executable code for {resource_type}. Transfer a description of a capability, available as an OWL ontology in Turtle syntax, into executable {language} {framework} code based on the control options from the context description.\n\nContext: {context}"
prompt_template = ChatPromptTemplate.from_messages(
    [("system", system_template), ('user', "{ontology}")]
)

ontology_path = os.path.join(base_dir, "../../capability-models", "set-velocity.ttl")
with open(ontology_path, "r") as f:
    ontology = f.read()


query = f"Generate {language} {framework} code from an OWL ontology in Turtle syntax for a {resource_type}.  {ontology}"
retrieved_docs = vector_store.similarity_search(query)
docs_content = "\n\n".join(doc.page_content for doc in retrieved_docs)
prompt = prompt_template.invoke({"language": language, "framework": framework, "resource_type": resource_type, "context": docs_content, "ontology": ontology})
# prompt = prompt.invoke({"question": question, "context": docs_content})
answer = llm.invoke(prompt)

print(answer)
