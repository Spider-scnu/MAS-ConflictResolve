import os
import random
from numpy.random import randint
from time import sleep
# get pwd
pwd = os.path.dirname(os.path.realpath(__file__))

cmd1 = "python " + "./get_new_task_file.py"
cmd2 = "python " + "./playground_system_class_lib.py"
cmd3 = "python " + "./get_new_init_position_file.py"

for i in range(5):
    os.system(cmd1)
    sleep(1)
    os.system(cmd3)
    sleep(1)
    os.system(cmd2)
