import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints: list, agent: int) -> dict:
    """" Creates a time keyed dictionary with all the constraints belonging to a certain agent. Can be vertex or edge
    constraints. The constraint types are differentiated by length of 'loc' keyed items in the constraints input
    list. So the returned dictionary has the timestep as keys, each with a list of constraints.

    :param constraints - the list of constrained dictionaries.
    :param agent - the id of the agent for which the constraint table must be created

    :return The time-keyed constraint dictionary for the specific agent.
    """
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraint_dict = {}

    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['timestep'] not in constraint_dict.keys():  # if constraint timestep not in dict, add it
                constraint_dict[constraint['timestep']] = [constraint]
            else:  # if constraint timestep already in dict, append to the list of constraints.
                constraint_dict[constraint['timestep']].append(constraint)

    return constraint_dict


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc: tuple, next_loc: tuple, next_time: int, constraint_table: dict) -> bool:
    """"
    Checks if the agents intended move is allowed for a specific state or not.

    :param curr_loc: the current location of the agent
    :param next_loc: the aimed location for the agent
    :param next_time: the next timestep
    :param constraint_table: the time-keyed dictionary with agent constraints.

    :return boolean
    """

    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    try:
        for constraint in constraint_table[next_time]:  # loop through all constraints active for the next_time
            if constraint['loc'] == next_loc or constraint['loc'] == [curr_loc, next_loc]:
                # Check if agents move is constrained via vertex constrained or edge constraint.
                return True
    except KeyError:  # if there are no constraints for the next_time: pass (and thus return false)
        pass

    try:  # check if there is a vertex constraint due to already finished agents
        for constraint in constraint_table[-1]:
            if constraint['start_time'] <= next_time and constraint['loc'] == next_loc:
                return True
    except KeyError:
        pass

    return False


def push_node(open_list, node):
    hashv = hash(repr(node))
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], hashv, node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def goal_constrained(goal_loc, curr_time, constraint_table):
    keys = [key for key in constraint_table.keys() if key > curr_time]

    for key in keys:
        if goal_loc in constraint_table[key]:
            return True

    return False


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    constraint_dict = build_constraint_table(constraints=constraints, agent=agent)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
    push_node(open_list, root)
    # closed_list[((root['loc'], root['time']))] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['g_val'] > 20 * root['h_val']:
            # if g_val exceeds root's h_val by a lot, the path cannot be found and the code should stop.
            break

        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and not goal_constrained(goal_loc, curr['time'], constraint_dict):
            return get_path(curr)

        for dir in range(5):
            if dir < 4:
                child_loc = move(curr['loc'], dir)
                if my_map[child_loc[0]][child_loc[1]]:  # filters out obstacles
                    continue
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'parent': curr,
                         'time': curr['time'] + 1}
            else:
                child_loc = curr['loc']
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'parent': curr,
                         'time': curr['time'] + 1}

            # Only push child node in open_list if child note doesn't violate constraints:
            if not is_constrained(curr_loc=curr['loc'],
                                  next_loc=child['loc'],
                                  next_time=child['time'],
                                  constraint_table=constraint_dict):
                if (child['loc'], child['time']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['time'])]
                    if compare_nodes(child, existing_node):
                        # closed_list[(child['loc'], child['time'])] = child
                        push_node(open_list, child)
                else:
                    # closed_list[(child['loc'], child['time'])] = child
                    push_node(open_list, child)

        closed_list[(curr['loc'], curr['time'])] = curr

    return None  # Failed to find solutions
