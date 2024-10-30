from langchain_core.tools import tool

from utils.tcp_server import call_server


@tool
def scan(zone: str) -> str:
    """ Use this to find up-to-date information about zones and objects in the world when asked"""
    command = (f"""set goal (and(robot_at burger {zone.strip()}))
                run
               get problem instances
               get problem predicates""")
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
    """Use this to see the pddl domain to check sintax of the permitted predicates and durative actions"""
    return call_server(b'get domain')

@tool
def get_plan() -> str:
    """ get the plan for the current goal"""
    return call_server(b'get plan')


def get_problem_predicates() -> str:
    """ Use this to get the stored predicates of the world,
    this knowledge may be outdated and need to be refreshed with a scan """
    return call_server(b'get problem predicates')


@tool
def get_problem_instances() -> str:
    """ Use this to get the stored instances of the world,
    this knowledge may be outdated and need to be refreshed with a scan """
    return call_server(b'get problem instances')


def run() -> str:
    """ run a plan """
    return call_server(b"""run""")
