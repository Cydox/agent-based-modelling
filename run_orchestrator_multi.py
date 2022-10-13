# !/usr/bin/python
import argparse
import glob
import itertools

from orchestrate_simulations import Orchestrator
from library_open_simulation_config import *
SOLVER = "CBS"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print(f"***Import an instance: {file}***")
        my_map, agent_groups, group_sizes, start_groups, goal_groups = import_mapf_instance(file)
        print_mapf_instance(my_map, start_groups, goal_groups)

        for size_combination in itertools.product(*[range(*group_size) for group_size in group_sizes.values()]):
            num_agents = {agent_groups[i]: size_combination[i] for i in range(len(agent_groups))}

            simulation_orchestrator = Orchestrator(
                map=my_map,
                num_agents=num_agents,
                start_groups=start_groups,
                goal_groups=goal_groups,
                planner=args.solver
            )

            for i in simulation_orchestrator:  # TODO: implement multi threading
                simulation_orchestrator.run(i)

            simulation_orchestrator.save_results()
            print('complete')

        print('complete complete')