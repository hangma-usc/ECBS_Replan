import argparse

def read_groups(file_name, offset=[1,1]):
    groups = dict()
    file = open(file_name, 'r')
    row = offset[1]
    for line in file:
        column = offset[0]
        for char in line:
            if str.isdigit(char):
                group = int(char)
                if group in groups:
                    groups[group].append([column, row])
                else:
                    groups[group] = [[column, row]]
            column += 1
        row += 1
    return groups

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("formation_start", help="input file containing the start formation")
    parser.add_argument("formation_end", help="input file containing the end formation")
    parser.add_argument("offset_start", help="offset vector for start formation")
    parser.add_argument("offset_end", help="offset vector for start formation")
    parser.add_argument("agents_file", help="output agents file")
    args = parser.parse_args()

    offset_start = [int(x) for x in args.offset_start.split(",")]
    offset_end = [int(x) for x in args.offset_end.split(",")]

    groups_start = read_groups(args.formation_start, offset_start[:-1])
    groups_end = read_groups(args.formation_end, offset_end[:-1])

    f = open(args.agents_file, 'w')
    f.write("{}\n".format(len(groups_start)))
    numAgents = 0
    for i in range(1, len(groups_start)):
        f.write("{},".format(len(groups_start[i])))
        numAgents += len(groups_start[i])
    f.write("{}\n".format(len(groups_start[len(groups_start)])))
    numAgents += len(groups_start[len(groups_start)])
    f.write("{}\n".format(numAgents))
    for i in range(1, len(groups_start)+1):
        for j in range(0, len(groups_start[i])):
            f.write("{},{},{},{},{},{}\n".format(
                groups_start[i][j][0], groups_start[i][j][1], offset_start[2],
                groups_end[i][j][0], groups_end[i][j][1], offset_end[2]
                ))









