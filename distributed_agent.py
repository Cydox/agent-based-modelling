from single_agent_planner import a_star
from cbs import detect_collision, get_location
# from run_experiments import print_mapf_instance

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
        self.start = start
        self.goal = goal
        self.heuristics = heuristics

        self.path_history = [start]
        self.location = start
        self.start = start
        self.goal = goal
        self.neighbors = []

        self.plan = self.__a_star([])
        self.planned_cost = len(self.plan)

    def time_step(self):
        try:
            self.path_history.append(self.plan[0])
            self.location = self.plan[0]
            self.plan = self.plan[1:]
        except IndexError:
            self.path_history.append(self.location)

    def set_neighbors(self, neighbors):
        self.neighbors = neighbors

    def update_plan(self):
        for n in self.neighbors:
            (col_loc, col_time) = self.__detect_collision(n)
            if not col_loc is None:
                self.__resolve_conflict(n, col_loc, col_time)
        
    def __detect_collision(self, n) -> tuple:
        try:
            collision = detect_collision(self.plan, n.plan)
            # detect_collision returns col_loc and time to collision. Time is currently one to low, but is no problem
        except IndexError:
            try:
                collision = detect_collision(self.plan, n.path_history)
            except IndexError:
                collision = detect_collision(self.path_history, n.plan)
        return collision

    def __resolve_conflict(self, n, col_loc, col_time):
        other_agents = (set(n.neighbors) | set(self.neighbors)) - {self} - {n}

        # option 1: replan own plan
        constraints = self.__generate_constraints(other_agents | {n})
        #
        # vertex_constraints = [constraint['loc'] for constraint in constraints if type(constraint['loc']) is tuple]
        # print_mapf_instance(self.my_map, starts=vertex_constraints[:10], goals=vertex_constraints)
        resolution_1 = self.__a_star(constraints, self.location)

        # option 2: replan other agent's plan
        constraints = n.__generate_constraints(other_agents | {self})
        resolution_2 = n.__a_star(constraints, n.location)

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
            self.plan = resolution_1[1:]
        else:
            n.plan = resolution_2[1:]

    def __generate_constraints(self, agent_list) -> list:
        """"
        Returns the constraints based on the planned locations of agents.
        :param agent_list - The agents for which constraints should be generated.
        """

        constraints = []

        for agent in agent_list:
            agent_locations = [agent.location] + agent.plan
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

    def __a_star(self, constraints, start=None):
        if start is None:
            start = self.start
        return a_star(self.my_map, start, self.goal, self.heuristics, self, constraints=constraints)


