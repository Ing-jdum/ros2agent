# Create a Langchain Tool object with the chain
from langchain_openai import ChatOpenAI
from dotenv import load_dotenv
from langchain_core.messages import HumanMessage
from langgraph.checkpoint.memory import MemorySaver
from langgraph.prebuilt import create_react_agent
import os

from tools.problem_expert import set_piece, set_piece_at_predicate, get_plan, run, get_problem_predicates, \
    get_problem_instances, set_goal, get_domain, scan

# Load environment variables from .env file
load_dotenv()

# Get a specific environment variable
api_key = os.getenv('OPENAI_API_KEY')

model = ChatOpenAI(model="gpt-4o")

tools = [set_piece, scan, set_piece_at_predicate, get_domain, run, get_plan, get_problem_predicates, get_problem_instances,
         set_goal]
system_prompt = ("You are a bridge between a classical plannifier and an user. "
                 "The user will write you request in natural langauge and you will call the corresponding tools writing the needed pddl code accordingly"
                 "When an user request some action to be performed, give back the generated plan and also an interpretation in natural language"
                 " first and ask the user if he wishes to go on with the plan, only after user consent, you can call the run tool.")

memory = MemorySaver()
config = {"configurable": {"thread_id": "test-thread"}}

agent_executor = create_react_agent(model, tools, state_modifier=system_prompt, checkpointer=memory)


while True:
    print("write a command in natural langauge: ")
    user_input = input(str())

    response = agent_executor.invoke(
        {"messages": [HumanMessage(content=user_input)]},
        config,
    )

    print(response["messages"][-1].content)

