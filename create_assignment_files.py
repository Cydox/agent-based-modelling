import itertools


def create_file_instances(filename, group_sizes):
    """"Purpose of this file is to create file instances for number of agents in each group.

        The 'regular' file definition takes a range of agent numbers within the group. This function converts such a
        file such that it will define only one number of agents per each group.

    """

    new_filename = filename.split('/')[0] + '/assignments_prioritized/' + filename.split('/')[1].split('.')[0] + '_'
    file = open(filename, 'r')
    lines = file.readlines()

    size_combinations = itertools.product(*[range(*group_size) for group_size in group_sizes.values()])

    assignments = []
    for size_combination in size_combinations:
        new_assignment = {
            list(group_sizes.keys())[i]: size_combination[i] for i in range(len(size_combination))
        }

        if not [assignment for assignment in assignments if list(assignment.values()) == list(new_assignment.values())[::-1]]:
            assignments.append(new_assignment)

    for assignment in assignments:
        new_lines = []

        for line in lines:
            for agent_group in assignment:
                if line.startswith(agent_group):
                    list_line = list(line)
                    list_line[3] = assignment[agent_group]
                    list_line[5] = assignment[agent_group]
                    line = ''.join([str(val) for val in list_line])
            new_lines.append(line)

        new_file = open(new_filename+'_'.join([str(val) for val in list(assignment.values())]) + '.txt', 'w')

        new_file.writelines(new_lines)
        new_file.close()


if __name__ == '__main__':
    create_file_instances('instances/assignment_lvl_1.txt', {'a1': (1, 10), 'a2': (1, 10)})
    create_file_instances('instances/assignment_lvl_2.txt', {'a1': (1, 10), 'a2': (1, 10)})
    create_file_instances('instances/assignment_lvl_3.txt', {'a1': (1, 10), 'a2': (1, 10)})