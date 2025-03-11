import uvicorn
from fastapi import FastAPI, UploadFile, Form, File, HTTPException
from typing import Dict
from langchain_core.messages import BaseMessage
from report_handling.control_report_handling import ControlReportHandling
from report_handling.ros2_handling import ROS2ReportHandling
from prompt_handling.prompt_handling import PromptHandler
from cap2skill import Cap2Skill

# Initialize FastAPI app
app = FastAPI(title="Capability to Skill Generation with LLMs", version="1.0")

control_entities_handler: Dict[str, ControlReportHandling] = {}

prompt_handler = PromptHandler()

@app.post("/generate-skill/")
async def generate_skill(language: str = Form(...), 
                        framework: str = Form(...), 
                        resource_type: str = Form(...),
                        context_file: UploadFile = File(None),
                        context_name: str = Form(...),
                        ontology_file: UploadFile = File(...) 
                        ):
    """
    Endpoint for code generation from an OWL capability ontology.
    """

    # Check if a new context file is given 
    if context_file:
        context = await context_file.read()
        context = context.decode("utf-8")

        # Set the context handler based on the framework
        if framework == "ROS2":
            control_entities_handler[context_name] = ROS2ReportHandling(context, framework, resource_type, prompt_handler)
        else:
            raise HTTPException(status_code=400, detail=f"Unsupported framework: {framework}")

    # Use the existing context handler if it exists
    else:
        if context_name not in control_entities_handler:
            raise HTTPException(status_code=400, detail=f"Context handler for '{context_name}' not found.")
        context = control_entities_handler[context_name]

    # Read ontology file 
    ontology = await ontology_file.read()
    ontology = ontology.decode("utf-8")

    # Call code generation via cap2skill
    cap2skill = Cap2Skill(language, ontology, control_entities_handler[context_name], prompt_handler)
    llm_answer: BaseMessage = cap2skill.generate_skill_code()

    return {"skill_code": llm_answer.content}

def run(): 
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    run()
