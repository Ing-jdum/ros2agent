# Create a Langchain Tool object with the chain
from langchain_openai import ChatOpenAI
from dotenv import load_dotenv
from langchain_core.messages import HumanMessage
from langgraph.checkpoint.memory import MemorySaver
from langgraph.prebuilt import create_react_agent
import os

from tools.problem_expert import get_plan, run, get_problem_predicates, \
    get_problem_instances, set_goals, get_domain, scan
from utils.tcp_server import init

# Load environment variables from .env file
load_dotenv()

# Get a specific environment variable
api_key = os.getenv('OPENAI_API_KEY')

model = ChatOpenAI(model="gpt-4o")

tools = [scan, get_domain, run, get_plan, get_problem_predicates, get_problem_instances,
         set_goals]
system_prompt = ("You are a bridge between a classical plannifier and an user. "
                 "The user will write you request in natural language and you will call the corresponding tools writing the needed pddl code accordingly. "
                 "When an user requests some action to be performed, give back the generated plan and also an interpretation in natural language. "
                 "First, ask the user if he wishes to go on with the plan, only after user consent, you can call the run tool.")

memory = MemorySaver()
config = {"configurable": {"thread_id": "test-thread"}}

agent_executor = create_react_agent(model, tools, state_modifier=system_prompt, checkpointer=memory)
init()
print("write a command in natural langauge: ")
while True:
    user_input = input(str())

    response = agent_executor.invoke(
        {"messages": [HumanMessage(content=user_input)]},
        config)

    print(response["messages"][-1].content)
