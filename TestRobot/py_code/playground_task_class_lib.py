from task_class_lib import *

task = Task()
task.readTaskfromFile(path_dir="./Task.txt")

for i in range(task.total_number_of_tasks_in_pool):
    task.taskGenerator()
    print(task.task_id, "\t", task.target_position, "\t", task.total_number_of_tasks_assigned)
