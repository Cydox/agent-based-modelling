# !/usr/bin/python
import argparse
import glob
import itertools
import multiprocessing
import numpy as np

from orchestrate_simulations import Orchestrator, Case
from library_open_simulation_config import *
SOLVER = "CBS"



def worker(q_cases, q_results):
    while True:
        c = q_cases.get()
        case = Case(c[0], c[1], c[2], c[3])
        # print('running case')
        result = case.run()
        q_results.put(result)

def generator(q_cases: multiprocessing.Queue, q_results: multiprocessing.Queue, iterator: Orchestrator, n_initial, n_workers):
    
    total_queued = 0
    total_results = 0

    results = []

    for i in range(n_initial):
        c = iterator.__next__()
        q_cases.put(c)

    total_queued = total_queued + n_initial




    for i in range(n_initial - n_workers):
        result = q_results.get()
        iterator.store_result(result)
        total_results = total_results + 1



    result = q_results.get()
    iterator.store_result(result)
    total_results = total_results + 1


    

    for c in iterator:
        q_cases.put(c)
        total_queued = total_queued + 1

        result = q_results.get()
        iterator.store_result(result)
        total_results = total_results + 1



    while total_queued > total_results:

        result = q_results.get()
        iterator.store_result(result)
        total_results = total_results + 1




    print('done... ')
    print (total_queued)
    print (total_results)

    iterator.save_results()



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

            # for i in simulation_orchestrator:  # TODO: implement multi threading
            #     simulation_orchestrator.run(i)



            n_workers = 4
            n_initial = 10

            q_cases = multiprocessing.Queue()
            q_results = multiprocessing.Queue()


            workers = [multiprocessing.Process(target=worker, args=(q_cases, q_results,), daemon=False) for w in range(n_workers)]
            # worker = multiprocessing.Process(target=worker, args=(q_cases, q_results,), daemon=False)
            it = simulation_orchestrator
            gen = multiprocessing.Process(target=generator, args=(q_cases, q_results, it, n_initial, n_workers), daemon=False)

            # worker.start()
            [w.start() for w in workers]

            gen.start()
            print('join')
            gen.join()
            print('joined')
            # worker.kill()
            [w.kill() for w in workers]





            # simulation_orchestrator.save_results()
            print('complete')

        print('complete complete')