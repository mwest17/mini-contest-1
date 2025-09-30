## See myAgents.py and search.py for the solution code

# Problem:
This contest was about a multiagent pacman search with a heavy time constraint. The end goal is to eat all pellets on the map without running out of points (and to gain the high score once the game ends)

For each move taken, -0.4 pts, while each 1/1000 of a second spent computing is -1 pts.

This solution uses weighted A* to store a path to the best food for each pacman to eat. The heuristic used encourages states that are close to food, while discouraging states that are close to other pacmen's goal state