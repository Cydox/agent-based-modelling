# !/usr/bin/python
import argparse
import glob
import itertools
import multiprocessing
import queue
import numpy as np

from orchestrate_simulations import Orchestrator, Case
from library_open_simulation_config import *
SOLVER = "CBS"

def save_unfinished(case_set: set):
    f = open('./simulation_results/aborted.txt', 'w')

    for case in case_set:
        f.write(case_from_tuple(case).__repr__() + "\n")

    f.close()

def case_to_tuple(case):
    return (tuple(case['starts']), tuple(case['goals']), case['id'])

def case_from_tuple(t):
    return {'starts': list(t[0]), 'goals': list(t[1])}

def worker(q_cases, q_results):
    while True:
        c = q_cases.get()
        case = Case(c[0], c[1], c[2], c[3])
        case.id = c[0]['id']
        # print('running case')
        result = case.run()
        q_results.put(result)

import time as timer

def generator(q_cases: multiprocessing.Queue, q_results: multiprocessing.Queue, iterator: Orchestrator, n_initial, n_workers, min_run_time=300):

    start_time = timer.time()
    
    
    i = 0
    total_queued = 0
    total_results = 0

    case_set = set()

    results = []

    for i in range(n_initial):
        c = iterator.__next__()
        c[0]['id'] = i
        i += 1
        q_cases.put(c)
        case_set.add(case_to_tuple(c[0]))

    total_queued = total_queued + n_initial




    for i in range(n_initial - n_workers):
        result = q_results.get()
        iterator.store_result(result)
        case_set.remove(case_to_tuple({'starts': result[2], 'goals': result[3], 'id': result[5]}))
        total_results = total_results + 1

    while timer.time() - start_time < min_run_time:

        result = q_results.get()
 

        iterator.store_result(result)
        case_set.remove(case_to_tuple({'starts': result[2], 'goals': result[3], 'id': result[5]}))
        total_results = total_results + 1


        c = iterator.__next__()
        c[0]['id'] = i
        i += 1
        q_cases.put(c)
        case_set.add(case_to_tuple(c[0]))
        total_queued = total_queued + 1


    result = q_results.get()
    iterator.store_result(result)
    case_set.remove(case_to_tuple({'starts': result[2], 'goals': result[3], 'id': result[5]}))
    total_results = total_results + 1


    
    test = 0
    for c in iterator:
        c[0]['id'] = i
        i += 1
        q_cases.put(c)
        case_set.add(case_to_tuple(c[0]))
        total_queued = total_queued + 1

        result = q_results.get()
 

        iterator.store_result(result)
        case_set.remove(case_to_tuple({'starts': result[2], 'goals': result[3], 'id': result[5]}))
        total_results = total_results + 1


    print('decided to stop, let remaining workers finish')
    while total_queued > total_results:

        print(f'waiting for {total_queued - total_results} workers to finish')

        try:
            result = q_results.get(timeout=30*60)
        except queue.Empty:
            print('aborting remaining cases')
            break
        
        iterator.store_result(result)
        case_set.remove(case_to_tuple({'starts': result[2], 'goals': result[3], 'id': result[5]}))
        total_results = total_results + 1




    print('done... ')
    print (total_queued)
    print (total_results)
    print(len(case_set))

    save_unfinished(case_set)

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

            n_workers = 32
            n_initial = 200

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