import itertools
import os
import time as timer

from cbs import CBSSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats


class Case:
    def __init__(self, input, planner, map, sim_id, case_id):
        """"Initiates a Case instance. Used to run one certain simulation with a planner, map, and agent input."""
        self.input = input
        self.planner = planner
        self.map = map
        self.sim_id = sim_id
        self.id = case_id

    def run(self):
        input = self.input

        starts = input['starts']
        goals = input['goals']

        if self.planner == "CBS":
            solver = CBSSolver(self.map, starts, goals)
        elif self.planner == "Prioritized":
            solver = PrioritizedPlanningSolver(self.map, starts, goals)
        elif self.planner == "Distributed":
            solver = DistributedPlanningSolver(self.map, starts, goals)
        else:
            raise RuntimeError("Unknown solver!")

        paths, total_cost, total_computation_time = solver.find_solution(print_results=False, return_costs=True)

        return total_cost, total_computation_time, starts, goals, self.sim_id, self.id


class Orchestrator:
    def __init__(self, my_map, num_agents: dict, start_groups: dict, goal_groups: dict, planner):
        """" Initiates orchestrator iterator. Determines all possible start-goal combinations that fit the inputs. So
        that is the cartesian product of all the possible start-goal combinations for each agent group.
        
        :param my_map: a map instance
        :param num_agents: the number of agents per agent group
        :param start_groups: the possible start location for each agent group
        :param goal_groups: the possible goal_groups for each goal group
        :param planner: planner type used in the simulations
        """

        self.start_time = timer.time()

        self.simulations_kpis = ['Cost', 'Computation time']

        # Required slope for the trend line through the each kpi's variation coefficient in order to stop running new
        # simulations:
        self.slope_threshold = 0.001

        # for every kpi, add 4 columns in the results table: kpi, kpi_mean, kpi_std, kpi_var
        col_names = ['starts', 'goals'] + list(
            itertools.chain(*[[kpi, kpi+'_var', kpi+'_mean', kpi+'_std'] for kpi in self.simulations_kpis])
        )
        self.simulation_results = pd.DataFrame(
            columns=col_names
        )
        self.simulation_results.index.name = 'simulation'

        self.simulation_inputs = []
        self.simulation_id = 0

        self.map = my_map
        self.planner = planner

        self.num_agents = num_agents
        self.agent_groups = list(start_groups.keys())
        self.file_name = \
            f'{" ".join([str(agent_group) + "-" + str(self.num_agents[agent_group]) for agent_group in self.agent_groups])} results'

        # format start and goal groups as dictionaries of arrays instead of lists
        # this is necessary to let np.random.choice work properly later on
        self.start_groups = {start_group: np.array(start_groups[start_group]) for start_group in start_groups}
        self.goal_groups = {goal_group: np.array(goal_groups[goal_group]) for goal_group in goal_groups}

    def store_result(self, results):

        (total_cost, total_computation_time, starts, goals, sim_id, id) = results

        print(total_cost, total_computation_time)
        # Build the dictionary that will eventually be the new row in the results table
        simulation_results = {
            self.simulations_kpis[0]: total_cost,
            self.simulations_kpis[1]: total_computation_time,
            'starts': str(starts),
            'goals': str(goals)
        }

        # Now for each KPI, store mean, std and variation coefficient in the dictionary as well
        for kpi in self.simulations_kpis:
            simulation_results[kpi+'_mean'] = np.append(self.simulation_results[kpi].to_numpy(),
                                                        simulation_results[kpi]).mean()
            simulation_results[kpi+'_std'] = np.append(self.simulation_results[kpi].to_numpy(),
                                                       simulation_results[kpi]).std()

            # Coefficient of variation in percentage:
            simulation_results[kpi+'_var'] = simulation_results[kpi+'_std'] / simulation_results[kpi+'_mean'] * 100

        # Store the results of this exact simulation as a ro win the results table.
        self.simulation_results = pd.concat([self.simulation_results, pd.Series(simulation_results).to_frame().T],
                                            ignore_index=True)

    def _is_stable(self):
        """" Checks if the results are stabilizing. If there have been 100 simulations and the variation coefficient
        of the simulation costs is stabilizing, no further simulations are needed.

        Mathematically, that means that the linear regressor through the last n rows should have a slope of almost 0
        """
        n = 50

        if timer.time() - self.start_time < 300:  # run for at least 5 minutes
            return False

        if self.simulation_results.shape[0] >= n:
            slopes = []
            for kpi in self.simulations_kpis:  # for each kpi check if slope of trend line is below threshold
                m = round(len(self.simulation_results) * 0.25)  # only check for last quarter of results
                x=self.simulation_results.index.to_list()[-m:]
                y=self.simulation_results[kpi+'_var'].to_list()[-m:]
                mask = ~np.isnan(x) & ~np.isnan(y)  # remove nans for runs that took to long
                regressor = scipy.stats.linregress(x=np.array(x)[mask],
                                                   y=np.array(y)[mask])
                slopes.append(regressor.slope)

            if self.simulation_id % 25 == 0:
                print(f'Simulating {self.simulation_id}th run for agents {self.num_agents}')
                print(f' - current slope of coefficient of variation = {[round(slope, 5) for slope in slopes]}')

                # save intermediate results to a csv
                try:
                    self.simulation_results.to_csv(f'simulation_results/{self.file_name}.csv.new', index=False)

                    os.rename(f'simulation_results/{self.file_name}.csv.new',
                              f'simulation_results/{self.file_name}.csv')
                except Exception as x:
                    print(f'Could not save intermediate results for due to: {x}')
                    pass

            # If all slopes fall within slope threshold:
            if len(
                    [slope for slope in slopes if -self.slope_threshold < slope < self.slope_threshold]
                   ) == len(self.simulations_kpis):
                return True

        return False

    def __iter__(self):
        return self

    def __next__(self):
        """"This is the iterator function. This is what parses the next input parameters for a simulation run.
        Stops if StopIteration is raised, so that is when the coefficient of variation for each KPI is stable."""

        if self._is_stable():
            raise StopIteration
        else:
            starts = []
            goals = []

            # For each agent group with n agents, pick n random start and goal locations.
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

            self.simulation_inputs.append(next_input)  # save the input for logging purposes
            self.simulation_id += 1
            return next_input, self.planner, self.map, self.simulation_id

    def save_results(self):
        """"Stores the results of the simulations in a .csv file and a plot figure. """
        # save the table with results
        self.simulation_results.reset_index(inplace=True)
        self.simulation_results.to_csv(f'simulation_results/{self.file_name}_final.csv', index=False)

        # save the plot
        plt.figure()

        for kpi in self.simulations_kpis:
            plt.plot(self.simulation_results[kpi+'_var'], label=kpi)

        plt.xlabel('Number of simulations')
        plt.ylabel('Coefficient of variation [%]')
        plt.suptitle(f'Simulated {", ".join(str(val) for val in self.num_agents.values())} agents.')
        plt.title(f'Avg cost: {self.simulation_results[self.simulations_kpis[0]].mean():.2f} - '
                  f'Avg computation time {self.simulation_results[self.simulations_kpis[-1]].mean():.2f}')
        plt.legend()
        fig = plt.gcf()
        fig.set_size_inches(18.5, 10.5)

        plt.savefig(f'simulation_results/{self.file_name}.png',
                    dpi=100)
        plt.close()
