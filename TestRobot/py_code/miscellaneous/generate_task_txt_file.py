import random
from numpy.random import randint

num_of_tasks = 100
num_of_agents = 14


text = str(num_of_tasks) + "\n"

ub = 17 # upper bound
lb = 0 # lower bound

starting_pos_list = random.sample(range(lb, ub+1), num_of_agents)
target_pos_list = random.sample(range(lb, ub+1), num_of_agents)


for i in range(1, num_of_tasks/2 + 1):
    col1 = str(i)
    col2 = str(randint(0, 32))
    col3 = str(randint(0, 32))
    
    starting_position = str(starting_pos_list[(i % num_of_agents)]) + " 17"
    target_position = str(target_pos_list[(i % num_of_agents)]) + "17"

    line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
    text += (line + "\n")



starting_pos_list = random.sample(range(lb, ub+1), num_of_agents)
target_pos_list = random.sample(range(lb, ub+1), num_of_agents)

for i in range(1, num_of_agents + 1):
    col1 = str(i)
    col2 = str(randint(0, 32))
    col3 = str(randint(0, 32))
    starting_position = "0 " + str(starting_pos_list[(i % num_of_agents)])
    target_position = "0 " + str(target_pos_list[(i % num_of_agents)])

    line = col1 + " " + col2 + " " + col3 + " " + starting_position + " " + target_position
    text += (line + "\n")


with open("./NewTasks.txt", "w") as f:
    f.write(text)
f.close()
