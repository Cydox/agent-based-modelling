"""
This file contains the AgentDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

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



class AgentDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, environment, my_map, start, goal, heuristics):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.heuristics = heuristics

        self.plan = []
        self.path_history = []
        self.location = start
        self.planned_cost = 0
        self.start = start
        self.goal = goal
        self.neighbors = []

    def time_step(self):
        self.path_history.append(self.plan[0])
        self.plan = self.plan[1:]

    def set_neighbors(self, neighbors):
        self.neighbors = neighbors

    def update_plan(self):
        for n in self.neighbors:
            (col_loc, col_time) = self.__detect_collision(n)
            if not col_loc is None:
                self.__resolve_conflict(n, col_loc, col_time)
        
    def __detect_collision(self, n) -> tuple:
       return detect_collision(self.plan, n.plan)

    def __resolve_conflict(self, n, col_loc, col_time):
        other_agents = n.neighbors

        # option 1: replan own plan
        constraints = self.__generate_constraints(other_agents + [n])
        resolution_1 = self.__a_star(constraints)


        # option 2: replan other agent's plan
        constraints = self.__generate_constraints(other_agents + [self])
        resolution_2 = n.__a_star(constraints)

        # evaluate which resolution option is preferable



    def __generate_constraints(agent_list):
        pass

    def __initial_planning(self):
        self.plan = self.__a_star([])
        self.planned_cost = len(self.plan)

    def __a_star(self, constraints):
        return a_star(self.my_map, self.start, self.goal, self.heuristics, 0, constraints=constraints)


