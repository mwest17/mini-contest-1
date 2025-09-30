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
    targetFoods = set()

    def path_to_closest_dot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        problem = AnyFoodSearchProblem(gameState, self.index, gameState.getNumPacmanAgents())

        # Find path to the closest food using astar
        return search.astar(problem, myAgentHeuristic) #Use beam search instead of astar. Reduce number of states searched (therefore faster)

    def get_action(self, state):
        if len(self.actionList) == 0: # Only recompute when we have reached state 
            MyAgent.targetFoods.discard(self.prevPos)
            self.actionList = self.path_to_closest_dot(state)
            endPos = findEndPoint(state.get_pacman_position(self.index), self.actionList) # Store this off? Prob should
            self.prevPos = endPos
            MyAgent.targetFoods.add(endPos)      # We should remove endPos to make it only store current goals  

        return self.actionList.pop(0)


    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """
        # Calculate optimal paths with all pacman in mind, then store. Only return next option then.
        self.actionList = []
        self.prevPos = None
        return


def findEndPoint(pos, actions):
    x, y = pos
    for act in actions:
        if act == 'North':
            x, y = x, y + 1
        elif act == 'East':
            x, y = x + 1, y
        elif act == 'South':
            x, y = x, y -1
        elif act == 'West':
            x, y = x - 1, y
    return (x,y)

# On Matthew's laptop:
#~890 pacman.py
#~615 autograder.py
def myAgentHeuristic(state,  # state is position of only our pacman
                    problem): 
    # Should check if state is close to another pacman's chosen 

    # We want to avoid recomputing, so we want to be far from other pacmen's chosen
        # What about when other pacmen are going for only food left. Costs us extra compute
            # Check if there are less food than pacmen, if there are, then don't weight based on this part

    # Do we even care about other pacmen in this scenario?? I think prob not. We only care about where they will be going
        # and we want to avoid going in the same area.

    # list of goal pellets
    # our pos
    # for all food pellets
        # Find our manhattan distance
        # Find distance of pellet from all other pacmen chosen pellets
        # Find the chosen that is the closest to the other pellets
        # Compute weighted ratio between the two

    # if numPacmen > pellets
        # Do we want to just the min dist to food pellet (to have them just go)?? 
        # If we do it right, could save on final few turns compute cost. Since we don't need to calc of every pacmen to other goal pellets

    food = problem.food.asList()

    cost_list = [abs(state[0] - f[0]) + abs(state[1] - f[1]) for f in food]
    
    if problem.numPacmen < len(food) and len(MyAgent.targetFoods) > 0: # We want to avoid where other pacmen are going
        # Weighted distance
        dist_to_other_goals = min([abs(state[0] - goal[0]) + abs(state[1] - goal[1]) for goal in MyAgent.targetFoods])
        cost = min(cost_list) / (0.015*dist_to_other_goals + 1)
    else:
        # Absolute distance
        cost = min(cost_list)
    return 42*cost # The meaning of life



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
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.get_pacman_position(agent_index)
        self.costFn = lambda x: 1
        
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

        self.agent_index = agent_index
        self.numPacmen = numPacmen

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

