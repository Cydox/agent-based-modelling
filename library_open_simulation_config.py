from pathlib import Path

import numpy as np


def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and
    a number for the start location of each agent.

    Example:
        @ @ @ @ @ @ @
        @ 0 1 . . . @
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
    """

    for agent_group in starts.keys():
        print(f'=== AGENT GROUP {agent_group} ===')
        print('Unique start locations')
        print_locations(my_map, starts[agent_group], char='s')

        print('Unique goal locations')
        print_locations(my_map, goals[agent_group], char='g')


def print_locations(my_map, locations, char):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = char
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] == char:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1-LineX: starting either with . or @: defines the shape of the rectangular world
        Line starting with 'a': defines the minimum and maximum number of agents to simulate of an agent group
        Line starting with 's': defines the vertices of the rectangle defining all starting nodes of an agent group
        Line starting with 'g': defines the vertices of the rectangle with all goal nodes of an group

        The agent group is defined by the symbol following the 'a', 's' or 'g' symbol. Can be 1 character long.

    Example:
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        a1 2 4           # run simulations with 2 to 4 agents in agent group one
        s1 1 1 1 3      # agent group one has starting rectangle spanning from (1, 1) to (1, 3)
        g1 1 4 1 3      # agent group one has goal rectangle spanning from (1, 4) to (1, 3)



    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')

    lines = [line.rstrip('\n') for line in f.readlines()]  # split all lines

    # define map:
    map_rows = [line.split(' ') for line in lines if line[0] in ['@', '.']]
    map = []

    for row in map_rows:
        map.append([])
        for cell in row:
            if cell == '@':
                map[-1].append(True)
            elif cell == '.':
                map[-1].append(False)

    # define agent start & goal groups
    agent_groups = [line[1] for line in lines if line.startswith('a')]

    # define all possible goal and start pairs
    group_sizes = {}
    start_groups = {}
    goal_groups = {}

    for agent_group in agent_groups:
        group_starts = []
        group_goals = []

        min_a, max_a = [int(num) for num in [line for line in lines if line.startswith('a' + agent_group)][0].split(' ')[1:]]
        max_a += 1  # this is necessary since python is end exclusive while people think end inclusive.

        sy1, sx1, sy2, sx2 = [int(num) for num in [line for line in lines if line.startswith('s' + agent_group)][0].split(' ')[1:]]
        gy1, gx1, gy2, gx2 = [int(num) for num in [line for line in lines if line.startswith('g' + agent_group)][0].split(' ')[1:]]

        for x in range(sx1, sx2 + 1):
            for y in range(sy1, sy2 + 1):
                group_starts.append((y, x))

        for x in range(gx1, gx2 + 1):
            for y in range(gy1, gy2 + 1):
                group_goals.append((y, x))

        group_sizes[agent_group] = min_a, max_a
        start_groups[agent_group] = group_starts
        goal_groups[agent_group] = group_goals

    f.close()

    return map, agent_groups, group_sizes, start_groups, goal_groups
