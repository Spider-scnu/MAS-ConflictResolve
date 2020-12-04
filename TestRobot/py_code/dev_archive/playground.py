from MultiAgentLib import *
from mas_utils import *

import time

# for i in range(50):
#     random_fake_position_data_generator(num_agent, map_size)


'''
Test Point class (removed)
'''

# pt1 = Point(1, 2)
# print(pt1.x, pt1.y)

# pt1.up()
# print(pt1.x, pt1.y, pt1.obstacle)

# pt1.is_obstacle()
# print(pt1.x, pt1.y, pt1.obstacle)


'''
Test Map class
'''

imap = Map()
imap.getMapReady4System(map_topology_preview=False)

'''
Test Task class
'''

task = Task()
task.readTaskfromFile() # load tasks from file
# for i in range(1, 105):
#     task.taskGenerator()
#     print("Assigned:", task.total_number_of_tasks_assigned)
#     print(task.target_position)

# robot = Robot(0, imap)

# task.taskGenerator()
# robot.target_position = task.target_position

# robot.naivePathGenerator()
# print(robot.predicted_path)
# print(robot.next_position)


# # print(robot.suboptimalPathGenerator())
# # print(robot.all_available_paths)

# exempted_next_positions = [(1, 0), (0, 1)]

# robot.adaptivePathSelector(exempted_next_positions)
# print("Selected conflict-free suboptimal path")
# print(robot.predicted_path)
# print(robot.next_position)


# print("Robot path generation")

# create an MAS 
mas = System(task, imap)
mas.readInitPositionsfromFile()
mas.createRobotsAtInitialPositions()
mas.publishFirstTasks4AllRobots()

mas.initialPathGeneration4AllRobots()

mas.runOnce(35)