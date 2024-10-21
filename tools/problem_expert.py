from langchain_core.tools import tool

from utils.tcp_server import call_server


@tool
def scan(zone: str) -> str:
    """ make the robot go to a certain zone, one present in the instances
    so it can scan it and see what's there.
    The robot will automatically update instances and predicates. Always ask the user before calling this function"""
    command = (f"""set goal (and(robot_at burger {zone.strip()}))
                run
               get problem instances""")
    return call_server(command.encode('utf-8'))

@tool
def set_goals(goals: list[str]) -> str:
    """Set multiple goals based on the domain to achieve so the planner system can get a plan.
    The syntax will be <predicate> <parameters> for example: piece_at something location, another_predicate requirements goal.
    """
    formatted_goals = "".join([f"({goal.strip()})" for goal in goals])
    command = f""" set goal (and {formatted_goals}) 
                get problem goal """
    return call_server(command.encode('utf-8'))


@tool
def get_domain(goal: str) -> str:
    """problem domain containing functions, syntax and expected parameters."""
    return call_server(b'get domain')

@tool
def get_plan() -> str:
    """ get the plan for the current goal"""
    return call_server(b'get plan')


def get_problem_predicates() -> str:
    """ get the current predicates in the world"""
    return call_server(b'get problem predicates')


@tool
def get_problem_instances() -> str:
    """ get the current instances in the world"""
    return call_server(b'get problem instances')


def run() -> str:
    """ run a plan """
    return call_server(b"""run""")
