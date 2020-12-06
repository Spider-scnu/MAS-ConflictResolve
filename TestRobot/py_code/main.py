from system_class_lib import System
from task_class_lib import Task
from robot_class_lib import Robot
from record_class_lib import Record
from map_class_lib import Map

from time import sleep

imap = Map()
imap.getMapReady4System(map_topology_preview=False)

task = Task()
task.readTaskfromFile()

mas = System(task, imap)

mas.readInitPositionsfromFile()
mas.createRobotsAtInitialPositions()
mas.publishFirstTasks4AllRobots()
mas.initialPathGeneration4AllRobots()


max_running_time = 1000
timestep = 0
for t in range(max_running_time):
    print("\nGlobal time: ", t)
    for robot in mas.robots:


        print("Current pos: ", robot.current_position, "\tNext pos: ", robot.next_position, "\tTarget pos: ", robot.target_position, "\tStatus: ", robot.status)
    mas.runOnce(1, timestep)
    if (t == max_running_time - 1):
        print("Executing time exceeds max duration")
        break
    timestep += 1
sleep(2)