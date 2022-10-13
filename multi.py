import multiprocessing
import numpy as np




class case:
    def __init__(self, parameter) -> None:
        self.parameter = parameter
        self.result = None
    
    def run(self):

        tmp = self.parameter

        for i in range(100):
            tmp = tmp + 1

        self.result = tmp
        return self.result


class case_iterator:
    def __init__(self) -> None:

        self.case_lst = []

    def __iter__(self):
        return self
    
    def __next__(self):

        # print(self.case_lst)
        if len(self.case_lst) > 0:
            print(self.case_lst[0])

        if np.sum([x.result for x in self.case_lst if x.result]) > 10:
            raise StopIteration

        c = case(np.random.choice(range(10)))
        self.case_lst.append(c)
        return c



def worker(q_cases, q_results):
    while True:
        case = q_cases.get()
        print('running case')
        case.run()
        q_results.put(case)


def generator(q_cases: multiprocessing.Queue, q_results: multiprocessing.Queue, iterator: case_iterator, n_initial, n_workers):
    
    total_queued = 0
    total_results = 0

    results = []

    for i in range(n_initial):
        c = iterator.__next__()
        print(f'adding case {c.parameter}')
        q_cases.put(c)

    total_queued = total_queued + n_initial


    sum = 0

    for i in range(n_initial - n_workers):
        result = q_results.get()
        total_results = total_results + 1
        results.append(result)

        sum = sum + result.result

    result = q_results.get()
    total_results = total_results + 1
    results.append(result)

    sum = sum + result.result
    

    while sum < 1000000:
        c = iterator.__next__()
        print(f'adding case {c.parameter}')
        q_cases.put(c)
        total_queued = total_queued + 1

        result = q_results.get()
        total_results = total_results + 1
        results.append(result)

        sum = sum + result.result

    while total_queued > total_results:

        result = q_results.get()
        total_results = total_results + 1
        results.append(result)

        sum = sum + result.result


    print('done... ')
    print (total_queued)
    print (total_results)
    # print(results)

    


if __name__ == "__main__":

    n_workers = 2
    n_initial = 10

    q_cases = multiprocessing.Queue()
    q_results = multiprocessing.Queue()


    workers = [multiprocessing.Process(target=worker, args=(q_cases, q_results,), daemon=False) for w in range(n_workers)]
    # worker = multiprocessing.Process(target=worker, args=(q_cases, q_results,), daemon=False)
    it = case_iterator()
    gen = multiprocessing.Process(target=generator, args=(q_cases, q_results, it, n_initial, n_workers), daemon=False)

    # worker.start()
    [w.start() for w in workers]

    gen.start()
    print('join')
    gen.join()
    print('joined')
    # worker.kill()
    [w.kill() for w in workers]
