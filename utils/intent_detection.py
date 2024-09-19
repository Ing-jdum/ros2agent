import os

from dotenv import load_dotenv
from langchain_core.prompts import FewShotChatMessagePromptTemplate, ChatPromptTemplate
from langchain_openai import ChatOpenAI


# Load environment variables from .env file
load_dotenv()

# Get a specific environment variable
api_key = os.getenv('OPENAI_API_KEY')

model = ChatOpenAI(model="gpt-4o-mini", temperature=0.0)

examples = [
    {"input": "where is the hammer?", "output": "QUESTION"},
    {"input": "where is the robot?", "output": "QUESTION"},
    {"input": "what is the name of the robot", "output": "QUESTION"},
    {"input": "is there a drill?", "output": "QUESTION"},
    {"input": "move the robot to the table one", "output": "ACTION"},
    {"input": "transport the drill to the table 04", "output": "ACTION"},
    {"input": "move the robot around", "output": "ACTION"},
    {"input": "move the robot and fetch the drill that is in the table", "output": "ACTION"}
]

# This is a prompt template used to format each individual example.
example_prompt = ChatPromptTemplate.from_messages(
    [
        ("human", "{input}"),
        ("ai", "{output}"),
    ]
)
few_shot_prompt = FewShotChatMessagePromptTemplate(
    example_prompt=example_prompt,
    examples=examples,
)

final_prompt = ChatPromptTemplate.from_messages(
    [
        ("system",
         "You are a intent detection specialized AI, you have to choose the closest intent to the ones in the list: "
         " 1- ACTION 2- QUESTION. You can also reply don't know if you don't know the answer."),
        few_shot_prompt,
        ("human", "{input}"),
    ]
)

chain = final_prompt | model

def intent_detection(user_input: str) -> str:
    """Get user intent."""
    return chain.invoke({"input": user_input}).content


