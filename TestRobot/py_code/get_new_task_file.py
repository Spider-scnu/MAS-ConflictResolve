import os
import random
from numpy.random import randint
# generate new tasks
num_of_agents = 16
num_of_tasks = 16

text = str(num_of_tasks) + "\n"

ub = 18 # upper bound
lb = 0 # lower bound

starting_pos_list = random.sample(range(lb, ub+1), num_of_agents)
target_pos_list = random.sample(range(lb, ub+1), num_of_agents)


for i in range(1, int(num_of_tasks/2) + 1):
    col1 = str(i)
    col2 = str(randint(0, 32))
    col3 = str(randint(0, 32))
    
    starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 19"
    target_position = str(target_pos_list[(i % num_of_agents)]) + " 19"

    line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
    text += (line + "\n")


starting_pos_list = random.sample(range(lb, ub+1), num_of_agents)
target_pos_list = random.sample(range(lb, ub+1), num_of_agents)

for i in range(int(num_of_tasks/2) + 1, num_of_tasks + 1):
    col1 = str(i)
    col2 = str(randint(0, 32))
    col3 = str(randint(0, 32))
    
    starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 0"
    target_position = str(target_pos_list[(i % num_of_agents)]) + " 0"

    line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
    text += (line + "\n")


with open("./Task.txt", "w") as f:
    f.write(text)
f.close()

