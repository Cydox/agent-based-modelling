from single_agent_planner import a_star
from cbs import detect_collision, get_location

# TODO : prepend curr loc to plan

class AgentDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.location = start
        self.goal = goal
        self.heuristics = heuristics

        self.path_history = [start]
        self.neighbors = []

        self.plan = self.__a_star()
        self.planned_cost = len(self.plan)

    def time_step(self):
        """"At every time step, move to the next plan step and remove this plan step from the plan.

        Note: the plan contains the current location (this helped readability of the code), so therefore the
        self.location is actually set to the second entry of the plan instead of first.
        """

        try:
            self.path_history.append(self.plan[1])
            self.location = self.plan[1]
            self.plan = self.plan[1:]
        except IndexError:
            self.path_history.append(self.location)

    def set_neighbors(self, neighbors):
        self.neighbors = neighbors

    def update_plan(self):
        """" Updates plan: if a future conflict is detected with any of the neighbors, resolve it."""
        for n in self.neighbors:
            (col_loc, col_time) = self.__detect_collision(n)
            if col_loc is not None:
                self.__resolve_conflict(n)
        
    def __detect_collision(self, n) -> tuple:
        """" Detects future collision between two agents plans """

        return detect_collision(self.plan, n.plan)

    def __resolve_conflict(self, n):
        other_agents = (set(n.neighbors) | set(self.neighbors)) - {self} - {n}

        # option 1: replan own plan
        constraints = self.__generate_constraints(other_agents | {n})
        resolution_1 = self.__a_star(constraints)

        # option 2: replan other agent's plan
        constraints = n.__generate_constraints(other_agents | {self})
        resolution_2 = n.__a_star(constraints)

        # no solution found
        if resolution_1 is None and resolution_2 is None:
            raise Exception

        # Evaluate which resolution option is preferable.
        # Note that this implementation varies from model properties. The model properties are defined as if agent 1
        # and agent 2 resolve the conflict at the same time individually. To save computation time, the conflict is
        # only being resolved once and possibly neighbor's plan is set.
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
            agent_locations = agent.plan
            for pseudo_time, loc in enumerate(agent_locations):
                constraints.append(  # apply vertex constraints
                    {
                        'agent': self,
                        'loc': loc,
                        'timestep': pseudo_time,
                    }
                )
                constraints.append(  # apply edge constraints
                    {
                        'agent': self,
                        'loc': [loc, agent_locations[pseudo_time - 1]],
                        'timestep': pseudo_time
                    }
                )
            constraints.append(
                {
                    'agent': self,
                    'loc': agent.goal,
                    'timestep': -1,
                    'start_time': len(agent.plan)
                }
            )

        return constraints

    def __a_star(self, constraints=None):
        """"Performs A* to create the plan for an agent. """
        if constraints is None:
            constraints = []

        return a_star(self.my_map, self.location, self.goal, self.heuristics, self, constraints=constraints)


