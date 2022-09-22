import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class IndependentSolver(object):
    """A planner that plans for each robot independently."""

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
        self.constraints = [{'agent': 1, 'loc': (1, 3), 'timestep': x} for x in range(10)]  # (y, x) from top left

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        ##############################
        # Task 0: Understand the following code (see the lab description for some hints)

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(my_map=self.my_map,
                          start_loc=self.starts[i],
                          goal_loc=self.goals[i],
                          h_values=self.heuristics[i],
                          agent=i,
                          constraints=self.constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

        ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result
