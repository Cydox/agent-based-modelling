import time as timer

import numpy as np

from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self, print_results=True, return_costs=False):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.process_time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            #print('before')
            #print(self.my_map, ", ",self.starts[i], ", ", self.goals[i], ", ", self.heuristics[i], ", ", i, ", ", constraints)
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            #print('after')
            if path is None:
                return [], np.nan, timer.process_time() - start_time
            result.append(path)

            for agent in range(self.num_of_agents):
                for time in range(len(path)):
                    if agent > i:  # only add constraints for future agents
                        constraints.append(  # apply vertex constraints
                            {
                                'agent': agent,
                                'loc': path[time],
                                'timestep': time,
                            }
                        )
                        constraints.append(  # apply edge constraints
                            {
                                'agent': agent,
                                'loc': [path[time], path[time-1]],
                                'timestep': time
                            }
                        )
                # apply vertex constraint for finished agents. 'start_time' is when the constraint should apply
                constraints.append(
                    {
                        'agent': agent,
                        'loc': path[-1],
                        'timestep': -1,
                        'start_time': len(path) - 1
                    }
                )


        self.CPU_time = timer.process_time() - start_time

        if print_results:
            print("\n Found a solution! \n")
            print("CPU time (s):    {:.2f}".format(self.CPU_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        if return_costs:
            return result, get_sum_of_cost(result), self.CPU_time
        else:
            return result
