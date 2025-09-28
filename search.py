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

# Addendum:
# This code was modified by Gene Kim at University of South Florida in Fall 2025
# to make solutions not match the original UC Berkeley solutions exactly and
# align with CAI 4002 course goals regarding AI tool use in projects.

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

util.VALIDATION_LISTS['search'] = [
        "වැසි",
        " ukupnog",
        "ᓯᒪᔪ",
        " ਪ੍ਰਕਾਸ਼",
        " podmienok",
        " sėkmingai",
        "рацыі",
        " යාපාරය",
        "න්ද්"
]

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def get_start_state(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def is_goal_state(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def get_successors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def total_action_cost(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


class Node():
    def __init__(self, state, actions, cost):
        self.state = state
        self.actions = actions
        self.cost = cost


def tinymaze_search(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def dfs(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.
    """
    return general_search(problem, 
                          util.LIFO())


def bfs(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    return general_search(problem, 
                          util.FIFO())


def ucs(problem: SearchProblem):
    """Search the node of least total cost first."""
    return general_search(problem, 
                          util.PriorityQueueWithFunction(lambda node: node.cost))


def null_heuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def astar(problem: SearchProblem, heuristic=null_heuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    return general_search(problem, 
                          util.PriorityQueueWithFunction(lambda node: node.cost + heuristic(node.state, problem)))
    

def general_search(problem: SearchProblem, fringe):
    """
    General algorithm for searching, fringe managing is defined by the data structure of each algorithm. 
    Each node is composed by (state, actions, cost)
    state= current state in the search problem (eg (x,y) coodinates)
    actions= list of actions to get to current state
    cost= to reach this state
    """
    start_position = problem.get_start_state()
    fringe.put(Node(start_position, [], 0))
    visited_set = [] # Is there any real benefit for us keeping this a set (other than random access speed)?

    while not fringe.is_empty():
        "Remove one node from the fringe"
        node = fringe.get()

        if node.state in visited_set:
            continue
        visited_set.append(node.state)

        if problem.is_goal_state(node.state):
            return node.actions
        "Loop the successors of current state"

        for next_state, next_action, next_cost in problem.get_successors(node.state):
        
            new_cost = node.cost + next_cost
            new_actions = node.actions + [next_action]
            new_node = Node(next_state, new_actions, new_cost)
            fringe.put(new_node)     

    return []  # No solution


