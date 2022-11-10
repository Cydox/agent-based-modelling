"""
This file contains the AgentDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star
from cbs import detect_collision, get_location





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

    def time_step(self):
        self.path_history.append(self.plan[0])
        self.location = self.plan[0]
        self.plan = self.plan[1:]

    def set_neighbors(self, neighbors):
        self.neighbors = neighbors

    def update_plan(self):
        for n in self.neighbors:
            (col_loc, col_time) = self.__detect_collision(n)
            if not col_loc is None:
                self.__resolve_conflict(n, col_loc, col_time)
        
    def __detect_collision(self, n) -> tuple:
       return detect_collision(self.plan, n.plan)

    def __resolve_conflict(self, n, col_loc, col_time):
        other_agents = n.neighbors

        # option 1: replan own plan
        constraints = self.__generate_constraints(other_agents + [n])
        resolution_1 = self.__a_star(constraints, self.location)


        # option 2: replan other agent's plan
        constraints = self.__generate_constraints(other_agents + [self])
        resolution_2 = n.__a_star(constraints, self.location)

        # no solution found
        if resolution_1 is None and resolution_2 is None:
            raise Exception

        # evaluate which resolution option is preferable
        if resolution_1 is None:
            cost_1 = float('inf')
        else:
            cost_1 = len(resolution_1) - len(self.plan)

        if resolution_2 is None:
            cost_2 = float('inf')
        else:
            cost_2 = len(resolution_2) - len(n.plan)

        if cost_1 < cost_2:
            self.plan = resolution_1
        else:
            n.plan = resolution_2

    def __generate_constraints(self, agent_list) -> list:
        """"
        Returns the constraints based on the planned locations of agents.
        :param agent_list - The agents for which constraints should be generated.
        """

        constraints = []

        for agent in agent_list:
            for time, loc in enumerate(agent.plan):
                constraints.append(  # apply vertex constraints
                    {
                        'agent': agent,
                        'loc': loc,
                        'timestep': time,
                    }
                )
                constraints.append(  # apply edge constraints
                    {
                        'agent': agent,
                        'loc': [loc, agent.plan[time - 1]],
                        'timestep': time
                    }
                )

        return constraints

    def __initial_planning(self):
        self.plan = self.__a_star([])
        self.planned_cost = len(self.plan)

    def __a_star(self, constraints, start=None):
        if start is None:
            start = self.start
        return a_star(self.my_map, start, self.goal, self.heuristics, 0, constraints=constraints)


