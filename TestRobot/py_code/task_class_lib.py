import numpy as np

class Task:
    def __init__(self):
        self.task_id = 0
        self.publish_time = []
        self.waiting_time = []

        self.total_number_of_tasks_in_pool = 0
        self.total_number_of_tasks_assigned = 0
        self.task_list_info = [] # a nested list of all tasks in the file

        self.starting_position = (0, 0)
        self.target_position = (0, 0)

        self.resetFlag = False

    def readTaskfromFile(self, path_dir="./../Task.txt"):
        try:
            task_file = open(path_dir, 'r')
            task_list = task_file.read()
            lines = task_list.split("\n")

            first_line = int(lines[0])

            while ("" in lines):
                lines.remove("")
            
            for line in lines[1:]:
                line = line.split(" ")
                task_i = []
                for l in line:
                    task_i.append(int(l))
                
                self.task_list_info.append(task_i)
            task_file.close()
            self.resetFlag = False
            print("Task file is loaded to task pool...")
            self.total_number_of_tasks_in_pool = len(self.task_list_info)
            print(self.total_number_of_tasks_in_pool)
        except IOError:
            print("Task file not found...")
    
    def taskGenerator(self, task_generation_mode="sequential"):
        if (self.resetFlag == True):
            return print("No tasks available, please read tasks from file...\n")
        
        else:
            if (task_generation_mode == "sequential"):
                try:
                    self.task_id += 1
                    #print('task-id', self.task_id)
                    if (self.task_id <= self.total_number_of_tasks_in_pool):
                        
                        task_item = self.task_list_info[self.task_id - 1]
                        self.publish_time = task_item[1]
                        self.waiting_time = task_item[2]

                        self.starting_position = (task_item[3], task_item[4])
                        self.target_position = (task_item[5], task_item[6])
                        self.total_number_of_tasks_assigned += 1
                        #print('-------------------------------------')
                    else:
                        print("All tasks are assigned. Please reset the Task object by task.taskReset()")
                except IndexError:
                    print("All tasks are assigned. Task list is now reset...\n")
                    self.taskReset() # or return to init position
                    self.total_number_of_tasks_assigned = 0
                    self.resetFlag = True
                
            if (task_generation_mode == "random"):
                self.task_id = np.random.randint(1, self.total_number_of_tasks_in_pool)

                task_item = self.task_list_info[self.task_id]
                self.publish_time = task_item[1]
                self.waiting_time = task_item[2]
                self.starting_position = (task_item[3], task_item[4])
                self.target_position = (task_item[5], task_item[6])

    def artificialTaskGenerator(self, map_x, map_y, num_of_robots, top2bot=None):
        # tbd
        pass

    def taskReset(self):
        self.task_id = 0
        self.publish_time = []
        self.waiting_time = []
        self.total_number_of_tasks_in_pool_in_pool = 0
        # self.task_list_info = []
        self.starting_position = (0, 0)
        self.target_position = (0, 0)