import os
import random
import numpy as np
from numpy.random import randint
# generate new tasks
num_of_agents = 16
num_of_tasks = 16 * 1000

text = str(num_of_tasks) + "\n"

ub = 18 # upper bound
lb = 0 # lower bound



for i in range(1, num_of_tasks + 1):
    #print()
    iteration_round = np.floor((i-1)/num_of_agents)
    if i % num_of_agents == 1:

        starting_pos_list = random.sample(range(lb, ub+1), num_of_agents   )
        target_pos_list = random.sample(range(lb, ub+1), num_of_agents)
    if i % num_of_agents < 8 :
        col1 = str(i)
        col2 = str(randint(0, 32))
        col3 = str(randint(0, 32))
        if iteration_round % 2 == 0:
            starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 19"
            target_position = str(target_pos_list[(i % num_of_agents)]) + " 19"
        else:
            starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 0"
            target_position = str(target_pos_list[(i % num_of_agents)]) + " 0"

        line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
        text += (line + "\n")
    else:
        col1 = str(i)
        col2 = str(randint(0, 32))
        col3 = str(randint(0, 32))
        if iteration_round % 2 == 0:
            starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 0"
            target_position = str(target_pos_list[(i % num_of_agents)]) + " 0"
        else:
            starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 19"
            target_position = str(target_pos_list[(i % num_of_agents)]) + " 19"

        line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
        text += (line + "\n")



with open("./../Task.txt", 'w') as f:
    f.write(text)
f.close()
