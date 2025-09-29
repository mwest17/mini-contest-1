# myAgents.py
# ---------------
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

from game import Agent
from searchProblems import PositionSearchProblem

import util
import time
import search

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def create_agents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

class MyAgent(Agent):
    """
    Implementation of your agent.
    """

    def path_to_closest_dot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.get_pacman_position(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)

        # Find path to the closest food using astar
        return search.astar(problem, myAgentHeuristic)

    def get_action(self, state):
        return self.path_to_closest_dot(state)[0]

    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """
        # Calculate optimal paths with all pacman in mind, then store. Only return next option then.
        return

def myAgentHeuristic(state,  # state is position of only our pacman
                    problem): 
    # Val - Manhattan distance to all other pacmen
    # Lower would be better than
    # States closer to a pellet but far away any other pacmen should have the lowest return value
    # For all food pellets, find manhattan distance to all pacman indexes. 
        # Find one with lowest dist to our index, but highest combined distance to other pacmen
        # Based on ordering of distances to other pacmen, make it so 1 is added to each that is closer to other pacmen??
    
    # Find distance to other pacmen
    # Optimal parallelization of pacmen activity would be for them all to be doing their own things in the corners. So 2width + 2length would be total dist.
    # So find actual manhattan distance between the pacmen. Then subtract from 2width + 2length

    # Dist to closest pellet + dist to other pacmen

    #print("State give to Heuristic:" + state)

    return 0
    
    # The heuristic is way too slow. Need to calculate a heuristic faster. Just heuristic for closest?
    # Should try to incoporate a caching/greedy approach to save information to avoid recalculation
    other_pacmen = problem.gameState.get_pacman_positions() # O(n) for n pacmen
    other_pacmen.pop(problem.agent_index)
    cost_list = []
    for food in problem.food.asList(): #O(f) for f pellets
        our_dist = abs(state[0] - food[0]) + abs(state[1] - food[1]) #O(1)
        min_other_dist = min([abs(pos[0] - food[0]) + abs(pos[1] - food[1]) for pos in other_pacmen]) # O(n) for n pacmen
        weighted_dist = our_dist / (min_other_dist + 1) # O(1)
        cost_list.append(weighted_dist)
        #cost_list.append(our_dist + 1.5*min_other_dist) 

    return 30*min(cost_list) # O(f) for f pellets


"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def path_to_closest_dot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.get_pacman_position(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)

        # Find path to the closest food using astar
        return search.astar(problem)

    def get_action(self, state):
        return self.path_to_closest_dot(state)[0]

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the path_to_closest_dot
    method.
    """

    def __init__(self, gameState, agent_index):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.get_pacman_position(agent_index)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

        self.gameState = gameState
        self.agent_index = agent_index

    def is_goal_state(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        # Goal is to just eat any food on the map
        for food in self.food.asList():
            if (x,y) == food:
                return True
        return False

