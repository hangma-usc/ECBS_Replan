import argparse
import json

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map2d", help="input file containing the 2d map")
    parser.add_argument("map3d", help="output map3d file")
    args = parser.parse_args()

    # dimensions = None
    data = dict()
    data["dimensions"] = []
    data["obstacles"] = []
    with open(args.map2d, "r") as map2d:
        r = -1
        for line in map2d:
            if r == -1:
                dim = line.split(',')
                data["dimensions"] = [int(dim[1]), int(dim[0]), 1]
            else:
                for c in range(0, len(line)):
                    if line[c] != "." and c < data["dimensions"][0] and r < data["dimensions"][1]:
                        data["obstacles"].append([c,r,0])
            r += 1

    with open(args.map3d, "w") as map3d:
        json.dump(data, map3d, indent=4)
