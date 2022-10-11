import itertools
import statistics

import scipy as scipy

from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver  # Placeholder for Distributed Planning
from visualize import Animation

from single_agent_planner import get_sum_of_cost
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class Orchestrator:
    def __init__(self, map, num_agents: dict, start_groups: dict, goal_groups: dict, planner):
        """" Initiates orchestrator iterator. Determines all possible start-goal combinations that fit the inputs. So
        that is the cartesian product of all the possible start-goal combinations for each agent group.
        
        :param map: a map instance
        :param num_agents: the number of agents per agent group
        :param start_groups: the possible start location for each agent group
        :param goal_groups: the possible goal_groups for each goal group
        :param planner: planner type used in the simulations
        """

        self.simulation_results = pd.DataFrame(
            columns=['cost', 'computation_time', 'current_simulation_avg', 'current_simulation_std_dev', 'variation_coefficient']
        )
        self.simulation_results.index.name = 'simulation'
        self.simulation_inputs = []

        self.simulation_id = 0

        self.map = map
        self.planner = planner

        self.num_agents = num_agents
        self.agent_groups = start_groups.keys()

        # format start and goal groups as dictionaries of arrays instead of lists
        # this is necessary to let np.random.choice work properly later on
        self.start_groups = {start_group: np.array(start_groups[start_group]) for start_group in start_groups}
        self.goal_groups = {goal_group: np.array(goal_groups[goal_group]) for goal_group in goal_groups}

        # # All the possible routes that 1 agent from a certain group can take:
        # # (store as dictionary containing arrays of cartesian product of all starts and goals)
        # self.agent_routes = {
        #     agent_group: np.array([
        #         route for route in itertools.product(start_groups[agent_group], goal_groups[agent_group])
        #     ]) for agent_group in self.agent_groups
        # }

    def run(self, input):
        starts = input['starts']
        goals = input['goals']

        if self.planner == "CBS":
            cbs = CBSSolver(self.map, starts, goals)
            paths, total_cost, total_computation_time = cbs.find_solution(print_results=False, return_costs=True)
        elif self.planner == "Prioritized":
            solver = PrioritizedPlanningSolver(self.map, starts, goals)
            paths, total_cost, total_computation_time = solver.find_solution(print_results=False, return_costs=True)
        # elif self.planner == "Distributed":
        #     solver = DistributedPlanningSolver(self.map, starts, goals)
        #     paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        self.simulation_results.loc[self.simulation_id, 'cost'] = total_cost

        mean = self.simulation_results['cost'].mean()
        std_dev = self.simulation_results['cost'].std()

        simulation_results = {
            'cost': total_cost,
            'computation_time': total_computation_time,
            'current_simulation_avg': mean,
            'current_simulation_std_dev': std_dev,
            'variation_coefficient':  std_dev / mean,
        }

        self.simulation_results.loc[self.simulation_id] = pd.Series(simulation_results)

    def _is_stable(self):
        """" Checks if the results are stabilizing. If there have been 100 simulations and the variation coefficient
        of the simulation costs is stabilizing, no further simulations are needed.

        Mathematically, that means that the linear regressor through the last n rows should have a slope of almost 0
        """
        n = 20

        if len(self.simulation_inputs) >= 40:
            regressor = scipy.stats.linregress(x=self.simulation_results.index.to_list()[-n:],
                                               y=self.simulation_results['variation_coefficient'].to_list()[-n:])

            if self.simulation_id % 20 == 0:
                print(f'Simulating {self.simulation_id}th run for agents {self.num_agents}')
                print(f' - current slope of coefficient of variation = {regressor.slope:.5f}')

            if -0.0005 < regressor.slope < 0.0005:
                return True

        return False

    def __iter__(self):
        return self

    def __next__(self):
        if self._is_stable():
            raise StopIteration
        else:
            # i = 0
            # while i < 100:
            starts = []
            goals = []

            for agent_group in self.agent_groups:
                num_agents = self.num_agents[agent_group]

                starts_indices = np.random.choice(len(self.start_groups[agent_group]), num_agents, replace=False)
                goals_indices = np.random.choice(len(self.goal_groups[agent_group]), num_agents, replace=False)

                starts += [tuple(start) for start in self.start_groups[agent_group][starts_indices]]
                goals += [tuple(goal) for goal in self.goal_groups[agent_group][goals_indices]]

            next_input = {
                'starts': starts,
                'goals': goals
            }

            # if next_input not in self.simulation_inputs:
            self.simulation_inputs.append(next_input)
            self.simulation_id += 1
            return next_input

            #     else:
            #         i += 1
            #
            # raise BaseException('error generating next unique input')

    def save_results(self):

        """"Stores the results of the simulations in a .csv file and a plot figure. """

        file_name = f'{" ".join([str(agent_group) + "-" +  str(self.num_agents[agent_group]) for agent_group in self.agent_groups])} results'

        # save the table with results
        self.simulation_results.to_csv(f'simulation_results/{file_name}.csv')

        # save the plot
        plt.figure()
        plt.plot(self.simulation_results['variation_coefficient'], label='Coefficient of variation')
        plt.xlabel('Number of simulations')
        plt.ylabel('Coefficient of variation')
        plt.title(f'Simulatied {", ".join(str(val) for val in self.num_agents.values())} agents. Avg cost: {self.simulation_results["cost"].mean():.2f}')
        plt.legend()
        fig = plt.gcf()
        fig.set_size_inches(18.5, 10.5)
        # plt.show()

        plt.savefig(f'simulation_results/{file_name}.png',
                    dpi=100)
        plt.close()


