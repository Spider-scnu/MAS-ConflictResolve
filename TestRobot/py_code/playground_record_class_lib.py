from record_class_lib import Record
from map_class_lib import Map
from robot_class_lib import Robot
from task_class_lib import Task
from time import sleep

my_map = Map()
my_map.getMapReady4System(map_topology_preview=False)

robots = [Robot(idx, my_map) for idx in range(2)]

task = Task()
task.readTaskfromFile(path_dir="./Task.txt")

# assign tasks
for robot in robots:
    robot.current_position = (robot.idx, 0)
    task.taskGenerator()
    robot.target_position = task.target_position
    robot.adaptivePathSelector(exempted_next_positions=[])

# generate paths

for robot in robots:
    print("Robot no.", robot.idx, "\tnext step: ", robot.next_position)
    print("Predicted path: ", robot.predicted_path, "\n")
    # print("All available paths:\n", robot.all_available_paths)

record = Record(robots, my_map.map_graph)
record.takeCurrentStepRecord(robots)
record.conflictCheck()
print(record.planned_next_step_conflict_flag)

sleep(2)