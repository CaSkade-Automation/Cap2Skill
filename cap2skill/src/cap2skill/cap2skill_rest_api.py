from fastapi import FastAPI, UploadFile, Form, File
import os
from cap2skill import Cap2Skill
from langchain_core.messages import BaseMessage
import uvicorn

# Initialize FastAPI app
app = FastAPI(title="Capability to Skill Generation with LLMs", version="1.0")

# Temporary upload directory
UPLOAD_DIR = "temp_uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)

def save_uploaded_file(uploaded_file: UploadFile) -> str:
    """Saves the uploaded file and returns its content."""
    file_path = os.path.join(UPLOAD_DIR, uploaded_file.filename)
    
    with open(file_path, "wb") as file:
        file.write(uploaded_file.file.read())

    # Dateiinhalt als String lesen
    with open(file_path, "r", encoding="utf-8") as file:
        content = file.read()

    return content

@app.post("/generate-skill/")
async def generate_skill(language: str = Form(...), 
                        framework: str = Form(...), 
                        resource_type: str = Form(...),
                        context_file: UploadFile = File(...),
                        ontology_file: UploadFile = File(...) 
                        ):
    """
    Endpoint for code generation from an OWL capability ontology.
    """

    # Save and read uploaded files
    context = save_uploaded_file(context_file)
    ontology = save_uploaded_file(ontology_file)

    # Call code generation via cap2skill
    cap2skill = Cap2Skill(language, framework, resource_type, context, ontology)
    llm_answer: BaseMessage = cap2skill.generate_skill_code()

    return {"skill_code": llm_answer.content}


def run(): 
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    run()
