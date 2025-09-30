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
        problem = AnyFoodSearchProblem(gameState, self.index, gameState.getNumPacmanAgents())

        # Find path to the closest food using astar
        return search.astar(problem, myAgentHeuristic) # Swap astar for greedy or beam??


    def get_action(self, state):
        if len(self.actionList) == 0: # Only recompute when we have reached the goal state 
            self.actionList = self.path_to_closest_dot(state)

        return self.actionList.pop(0)


    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """
        # Calculate optimal paths with all pacman in mind, then store. Only return next option then.
        self.actionList = []
        return


# On Matthew's laptop:
#~925 pacman.py
#~988 autograder.py
def myAgentHeuristic(state,  # state is position of only our pacman
                    problem): 
    # Incentivizes states that are closer to pellets
    # However it disincentivizes states that are near other pacmen
    # If we choose well enough, the pacmen can all go to unqiue pellets that are hard for other pacmen to reach
    # This avoids recomputation and makes the pacmen eat all the pellets quicker

    cost_list = []
    for f in problem.food:
        dist_food = abs(state[0] - f[0]) + abs(state[1] - f[1]) # Our distance to food
        min_other_dist = min([abs(pos[0] - f[0]) + abs(pos[1] - f[1]) for pos in problem.other_pacmen]) # Food distance to other pacmen
        weighted_dist = dist_food / (min_other_dist + 1) # Weight our distance to food by others distance
        cost_list.append(weighted_dist)

    return 42*min(cost_list) # The meaning of life * best weighted food



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

    def __init__(self, gameState, agent_index, numPacmen):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        #self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.get_pacman_position(agent_index)
        self.costFn = lambda x: 1
        
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

        self.agent_index = agent_index
        self.numPacmen = numPacmen

        self.food = gameState.getFood().asList()
        self.amountFood = len(self.food)

        self.other_pacmen = gameState.get_pacman_positions()
        self.other_pacmen.pop(agent_index)

    def is_goal_state(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        # Goal is to just eat any food on the map
        for f in self.food:
            if (x,y) == f:
                return True
        return False

