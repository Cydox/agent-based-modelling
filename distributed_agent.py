"""
This file contains the AgentDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

class AgentDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, environment, my_map, start, goal, heuristics):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.heuristics = heuristics

        self.plan = []
        self.path_history = []
        self.location = start
        self.planned_cost = 0
        self.start = start
        self.goal = goal
        self.neighbors = []

    def timestep():
        pass

    def set_neighbors(self, neighbors):
        self.neighbors = neighbors

    def update_plan():
        pass

    def __initial_planning(self):
        pass

    def __a_star(self, constraints):
        pass
