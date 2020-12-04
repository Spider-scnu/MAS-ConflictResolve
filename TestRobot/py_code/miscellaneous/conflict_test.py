# from mas_utils import nextPosTakeCurrentPos


# next_conflict_dict = {(0, 2): [0, 1], 
#                  (0, 6): [2, 3], 
#                  (0, 10): [4, 5], 
#                  (0, 14): [7], 
#                  (1, 13): [6]}

# # next_conflict_dict = {(0, 2): [0],
# #                       (0, 4): [1],
# #                       (0, 6): [2], 
# #                       (0, 15): [3], 
# #                       (0, 13): [4],
# #                       (0, 10): [5], 
# #                       (0, 16): [6],
# #                       (0, 14): [7]}


# current_position_dict = {(0, 1): 0,
#                          (0, 3): 1, 
#                          (0, 5): 2, 
#                          (0, 7): 3, 
#                          (0, 9): 4, 
#                          (0, 11): 5, 
#                          (1, 13): 6, 
#                          (0, 14): 7}


# # next conflict check
# if (any((len(item[1]) > 1) for item in next_conflict_dict.items())):
#     print("NEXT pos conf happens")

# else:
#     # next takes non-self current check
#     for item in next_conflict_dict.items():
#         next_pos = item[0]
#         idx = item[1][0]
#         try:
#             if (current_position_dict[next_pos] != idx):
#                 # print("Planned pos takes other robot's current pos")
#                 print("robot " + str(idx) + " next_pos" + str(next_pos) + " takes" + " robot " + str(current_position_dict[next_pos]) + " current pos")
#                 break
#         except KeyError:
#             pass

from collections import namedtuple
from time import sleep

rbt_log = namedtuple("rbt_log", ["idx", "current_pos", "next_pos", "target_pos"])

rbt_log_list = [
                rbt_log(idx=0, current_pos=(1, 1), next_pos=(2, 1), target_pos=(17, 7)),
                rbt_log(idx=1, current_pos=(0, 2), next_pos=(0, 1), target_pos=(17, 1)),
                rbt_log(idx=2, current_pos=(0, 6), next_pos=(0, 7), target_pos=(17, 12)),
                rbt_log(idx=3, current_pos=(0, 8), next_pos=(1, 8), target_pos=(17, 4)),
                rbt_log(idx=4, current_pos=(0, 9), next_pos=(0, 8), target_pos=(17, 13)),
                rbt_log(idx=5, current_pos=(0, 10), next_pos=(0, 9), target_pos=(17, 11)),
                rbt_log(idx=6, current_pos=(1, 13), next_pos=(2, 13), target_pos=(17, 6)),
                rbt_log(idx=7, current_pos=(0, 16), next_pos=(0, 17), target_pos=(17, 17))
            ]



# check this 
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



conflict_dict, current_position_dict = findConflictDictInfo(rbt_log_list)

print(conflict_dict)

print(current_position_dict)

sleep(2)