import itertools
import time as timer
import heapq
import random

import numpy as np

from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from copy import deepcopy

def detect_collision(path1: list, path2: list) -> tuple:
    """"For two anonymous paths, return the location and timestep for the first collision. That can be either a
    vertex collision or edge collision.

    :param path1: path of agent 1
    :param path2: path of agent 2

    :return (col_loc, col_time): col_loc is either a tuple for a vertex collision, or a list of two tuples for an edge
    collision. Both col_loc and time are None if no collisions found
    """
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    col_loc = None
    col_time = None

    for time in range(max(len(path1), len(path2))):
        a1_loc1 = get_location(path1, time)
        a1_loc2 = get_location(path1, time + 1)
        a2_loc1 = get_location(path2, time)
        a2_loc2 = get_location(path2, time + 1)

        if a1_loc1 == a2_loc1:
            col_loc = a1_loc1
            col_time = time
            break
        elif a1_loc1 == a2_loc2 and a2_loc1 == a1_loc2:
            col_loc = [a1_loc1, a1_loc2]
            col_time = time + 1
            break

    return col_loc, col_time


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    for a1, a2 in itertools.combinations(range(len(paths)), 2):
        col_loc, col_time = detect_collision(path1=paths[a1], path2=paths[a2])

        if col_loc is not None:
            collisions.append(
                {
                    'a1': a1,
                    'a2': a2,
                    'loc': col_loc,
                    'timestep': col_time
                }
            )

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    agents = [collision['a1'], collision['a2']]
    constraints = []

    if type(collision['loc']) is tuple:
        constraints.append({'agent': agents[0], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': agents[1], 'loc': collision['loc'], 'timestep': collision['timestep']})
    else:
        constraints.append({'agent': agents[0], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': agents[1], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']})

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []
        self.closed_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, print_results=True, return_costs=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        self.node_num = 0

        while len(self.open_list) > 0:
            parent = self.pop_node()

            if len(parent['collisions']) == 0:
                if print_results:
                    self.print_results(parent)
                if return_costs:
                    return parent['paths'], get_sum_of_cost(parent['paths']), timer.time() - self.start_time
                else:
                    return parent['paths']

            collision = parent['collisions'][0]
            constraints = standard_splitting(collision)

            for constraint in constraints:
                agent = constraint['agent']

                child = {'cost': 0,
                         'constraints': parent['constraints'] + [constraint],
                         'paths': deepcopy(parent['paths']),
                         'collisions': []}

                path = a_star(
                    self.my_map,
                    self.starts[agent],
                    self.goals[agent],
                    self.heuristics[agent],
                    agent,
                    child['constraints']
                )
                self.node_num += 1

                if path is None:
                    continue

                child['paths'][agent] = deepcopy(path)
                child['cost'] = get_sum_of_cost(child['paths'])
                child['collisions'] = deepcopy(detect_collisions(child['paths']))

                self.push_node(child)

            ##############################
            # Task 3.3: High-Level Search
            #           Repeat the following as long as the open list is not empty:
            #             1. Get the next node from the open list (you can use self.pop_node()
            #             2. If this node has no collision, return solution
            #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
            #                standard_splitting function). Add a new child node to your open list for each constraint
            #           Ensure to create a copy of any objects that your child nodes might inherit

        return root['paths'],

    def print_results(self, node):
        print("\n Found a solution! \n")

        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
