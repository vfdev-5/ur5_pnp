#!/usr/bin/env python
#
# Tiny library to read files saved by moveit commander with joint state
# 

import os
import json


def read(filepath):
    """Method to read input filepath with joint states:
    Example file contains joint states:
    ```
    use manipulator
    near_box = [0.414210766554 -0.975084129964 0.818676948547 -1.61239320437 -1.52766591707 0.000167779158801]
    home = [3.07549442165e-05 -1.56997307877 -4.41303162836e-05 -1.56990515225 -4.37613382936e-05 -6.03143178858e-05]
    drop = [1.2162232399 -0.528184239064 0.557434082031 -1.72803575197 -1.5297630469 0.00371136469766]
    pick = [0.410748451948 -0.647123161946 0.817443847656 -1.73179895083 -1.52772552172 0.000179763374035]
    ``` 

    Returns a dictionary with joint state names as keys and joint state list of values as values 
    """
    assert os.path.exists(filepath), "File is not found '{}'".format(filepath)
    output = {}
    with open(filepath, 'r') as h:
        max_lines_counter = 1000
        while max_lines_counter > 0:
            line = h.readline()
            if len(line) == 0:
                break
            max_lines_counter -= 1
            if "=" in line:
                splt = line.split(" = ")
                assert len(splt) == 2, "Problem with the line: {}".format(line)
                output[splt[0]] = json.loads(splt[1].replace(" ", ", "))
    return output


if __name__ == "__main__":
    
    test_filepath = "../../pick_and_drop.cmds"
    print(read(test_filepath))



