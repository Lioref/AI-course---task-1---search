# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

debug_ctr = 0


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    # problem is of type: searchAgents.PositionSearchProblem
    # it has attributes: getStartState() - which returns (x,y)

    # setup
    paths_dict = {}  # maps a child state key to a PathInfo object: it's parent state and to the action from the parent to the child
    frontier = util.Stack()
    start_node = Node(problem.getStartState(), None, 0)
    reached = set()

    if problem.isGoalState(problem.getStartState()):
        return []
    setup_first_step(problem, reached, frontier, start_node, paths_dict)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.get_state()):
            return generate_path_to_goal(goal_node=node, paths_map=paths_dict)
        for successor in problem.getSuccessors(node.get_state()):
            successor_node = Node(*successor)
            if not successor_node.get_state() in reached:
                update_paths_map(child=successor_node, parent=node, paths_map=paths_dict)
                frontier.push(successor_node)
        update_reached(reached, node)


def setup_first_step(problem, reached, frontier, start_node, paths_dict):
    for successor in problem.getSuccessors(problem.getStartState()):
        node = Node(*successor)
        update_paths_map(child=node, parent=start_node, paths_map=paths_dict)
        frontier.push(node)
    reached.add(problem.getStartState())


def update_paths_map(child, parent, paths_map):
    paths_map[child.get_state()] = PathInfo(parent.get_state(), child.get_action())


def update_reached(reached, node):
    reached.add(node.get_state())


def generate_path_to_goal(goal_node, paths_map):
    goal_to_start_path = []
    child_node = goal_node
    child_state = child_node.get_state()
    while has_parent(child_state, paths_map):
        goal_to_start_path.append(paths_map[child_state].get_action())
        child_state = paths_map[child_state].get_parent_state()
    goal_to_start_path.reverse()
    return goal_to_start_path


def has_parent(child_state, paths_map):
    return child_state in paths_map


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    frontier = util.Queue()
    paths_dict = {}  # maps a child state key to a PathInfo object: it's parent state and to the action from the parent to the child
    start_node = Node(problem.getStartState(), None, 0)
    reached = set()

    if problem.isGoalState(problem.getStartState()):
        return []
    frontier.push(start_node)
    reached.add(problem.getStartState())

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.get_state()):
            return generate_path_to_goal(goal_node=node, paths_map=paths_dict)
        for successor in problem.getSuccessors(node.get_state()):
            successor_node = Node(*successor)
            if successor_node.get_state() not in reached:
                update_paths_map(child=successor_node, parent=node, paths_map=paths_dict)
                update_reached(reached, successor_node)
                frontier.push(successor_node)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()

    start_node = Node(problem.getStartState(), None, 0)
    paths_dict = {}  # child state mapped to it's parent state and the action from the parent to the child
    paths_cost_dict = {start_node.get_state(): 0}  # node state mapped to path cost to node

    if problem.isGoalState(problem.getStartState()):
        return []
    frontier.push(start_node, 0)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.get_state()):
            return generate_path_to_goal(node, paths_dict)
        for successor in problem.getSuccessors(node.get_state()):
            successor_node = Node(*successor)
            new_cost = paths_cost_dict[node.get_state()] + successor_node.get_cost()

            if not_visited(successor_node, paths_cost_dict) or is_better_cost(new_cost, successor_node, paths_cost_dict):
                paths_cost_dict[successor_node.get_state()] = new_cost
                update_paths_map(child=successor_node, parent=node, paths_map=paths_dict)
                frontier.push(successor_node, new_cost)

def not_visited(node, paths_cost):
    return node.get_state() not in paths_cost

def is_better_cost(new_cost, node, paths_cost):
    return new_cost < paths_cost[node.get_state()]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()

    start_node = Node(problem.getStartState(), None, 0)
    paths_dict = {}  # child state mapped to it's parent state and the action from the parent to the child
    paths_cost_dict = {start_node.get_state(): 0 + heuristic(start_node.get_state(), problem)}  # node state mapped to path cost to node

    if problem.isGoalState(problem.getStartState()):
        return []
    frontier.push(start_node, 0)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.get_state()):
            return generate_path_to_goal(node, paths_dict)
        for successor in problem.getSuccessors(node.get_state()):
            successor_node = Node(*successor)
            new_cost = paths_cost_dict[node.get_state()] + successor_node.get_cost()
            heuristic_total_cost = new_cost + heuristic(successor_node.get_state(), problem)
            if not_visited(successor_node, paths_cost_dict) or is_better_cost(heuristic_total_cost, successor_node,
                                                                              paths_cost_dict):
                paths_cost_dict[successor_node.get_state()] = new_cost
                update_paths_map(child=successor_node, parent=node, paths_map=paths_dict)
                frontier.push(successor_node, heuristic_total_cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


class PathInfo:
    """ A class that defines the info needed to retrace the path after goal is reached """

    def __init__(self, parent_state, action, path_cost = 0):
        self._parent_state = parent_state
        self._action = action
        self._path_cost = path_cost

    def get_parent_state(self):
        return self._parent_state

    def get_action(self):
        return self._action

    def get_path_cost(self):
        return self._path_cost

    def set_path_cost(self, cost):
        self._path_cost = cost


class Node:
    """ A class that defines a node in the search tree """

    def __init__(self, state, action, cost):
        self._state = state
        self._action = action
        self._cost = cost

    def get_state(self):
        return self._state

    def get_action(self):
        return self._action

    def get_cost(self):
        return self._cost
