import numpy as np
import time 
import inspect

def random_position_generator(map_size):
    if len(map_size) != 2:
        return "map_size Error"
    else:
        row, col = map_size[0], map_size[1]

        x = np.random.randint(1, row)
        y = np.random.randint(1, col)

        return str(x) + "," + str(y)

def random_fake_position_data_generator(num_agent, map_size):
    info_per_step = str(num_agent) + "\n"

    for agent in range(1, num_agent+1):
        info_per_step = info_per_step + str(agent) + "," + random_position_generator(map_size) + "," + random_position_generator(map_size) + "\n"

    with open("./../Robot_Current_Position.txt", 'w') as f:
        f.write(info_per_step)
    f.close()
    print("Generating random positions...")
    time.sleep(2)
    return

def OneIdx2ZeroIdx(point):
    return (point[0] - 1, point[1] - 1)

def ZeroIdx2OneIdx():
    pass

def findConflictDictInfo(rbt_log_list):
    next_position_list = []
    current_position_dict = {}

    for robot_log in rbt_log_list:
        next_position_list.append(robot_log.next_pos)
        current_position_dict.update({robot_log.current_pos: robot_log.idx})
    # don't forget to pop() its own current_pos in exempted pos list

    conflict_dict = {}
    next_position_set = set(next_position_list)

    for item in next_position_set:
        conflict_robots_indices = [i for i, pos in enumerate(next_position_list) if pos == item]
        conflict_dict.update({item: conflict_robots_indices})

    # print(conflict_dict)
    return conflict_dict, current_position_dict


def writeRecords(log_every_step):
    with open("./../Log.txt", "a") as f:
        f.write(log_every_step)
    f.close()


def findMaxDepth(nested_list):
    max_depth = 1
    for l in nested_list:
        if (len(l) >= max_depth):
            max_depth = len(l)
    
    return max_depth

def check2typesConflicts(conflict_dict, current_position_dict):
    pass



def nextPosTakeCurrentPos(conflict_dict, current_position_dict):
    conflict_flag = False

    for item in conflict_dict:
        pos = item[0]
        indices = item[1]

    print(conflict_flag)
    return conflict_flag