import time as timer
from scipy.spatial import distance
import numpy as np

from single_agent_planner import compute_heuristics, get_sum_of_cost
from distributed_agent import AgentDistributed


class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.dist_threshold = 4  # the radius of any agent's local radar (in cell lengths)

        self.my_map = my_map

        self.map = my_map
        self.agents = [AgentDistributed(start=start,
                                        goal=goal,
                                        heuristics=compute_heuristics(my_map=my_map, goal=goal),
                                        my_map=my_map
                                        ) for start, goal in zip(starts, goals)]

        self.solved = False

    def find_solution(self, print_results=True, return_costs=False):
        """
        Finds paths for all agents from start to goal locations. 

        Parameters:
            :param print_results - whether to print the results or not
            :param return_costs - whether to return the cost and computation time

        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.process_time()  # using perf_counter as time.time() is bad practice

        try:
            while not self.solved:  # step through time until solved
                self.time_step()
        except Exception as x:
            print(x)
            computation_time = timer.process_time() - start_time
            if return_costs:
                return [], np.nan, computation_time
            else:
                return []

        computation_time = timer.process_time() - start_time
        result = [agent.path_history for agent in self.agents]  # list of agent paths.

        # Print final output
        if print_results:
            print("\n Found a solution! \n")
            print("CPU time (s):    {:.2f}".format(computation_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        if return_costs:
            return result, get_sum_of_cost(result), computation_time
        else:
            return result

    def time_step(self):
        """"
        Step through time by one time step. Update agent locations and let them update their plan for next time step.

        If all agent locations are at the goal location, mark solver state as solved.
        """

        # Set neighbors of all agents (implementation of local radar)
        for agent in self.agents:
            neighbors = [neighbor for neighbor in self.agents
                         if neighbor != agent and self._distance(neighbor, agent) <= self.dist_threshold]

            agent.set_neighbors(neighbors=neighbors)

        # With neighbors set, let agents update their plan.
        for agent in self.agents:
            agent.update_plan()

        # With every plan made, execute plans for one timestep.
        # (this is not combined with previous for-loop since all agents must make their plan at the same time.
        for agent in self.agents:
            agent.time_step()

        # if all agents at goal loc and have no more plans, then it is solved
        self.solved = all([(agent.location == agent.goal) & (len(agent.plan) == 1) for agent in self.agents])

    @staticmethod
    def _distance(agent1, agent2) -> float:
        """Calculates the euclidean distance between two agents."""
        return distance.euclidean(agent1.location, agent2.location)
