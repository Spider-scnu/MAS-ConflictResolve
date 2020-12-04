# for robot class testing
from time import sleep
from map_class_lib import Map
from task_class_lib import Task
from robot_class_lib import Robot

# get map ready
my_map = Map()
my_map.getMapReady4System(FW=True)

# load task file
task = Task()
task.readTaskfromFile(path_dir="./Task.txt")

# create robots
# num_of_robot = 8
multi_robot = []

robot = Robot(0, my_map)

# robot.current_position = (0, 0)

# get task from task pool, move to class System 
task.taskGenerator()
# robot.target_position = task.target_position
robot.target_position = (18, 17)

# robot.naivePathGenerator_FW()
# robot.naivePathGenerator_AStar()

# print(robot.all_available_paths)

# paths = robot.suboptimalPathGenerator(algorithm="AStar")

# for item in paths:
#     print(item)

# robot.adaptivePathGenerator(algorithm="AStar")

# for item in robot.all_available_paths.items():
#     print(item)


# neighbor nodes captured
# no_next_positions = [(18, 17)]

# no_next_positions = [(1, 0), (0, 1), (0, 1), (0, 0)]

# no_next_positions = [(0, 1)]
no_next_positions = []

robot.adaptivePathSelector(exempted_next_positions=no_next_positions)

for item in robot.all_available_paths.items():
    print(item)

robot.plotPath()

sleep(2)