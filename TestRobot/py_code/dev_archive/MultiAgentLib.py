from mas_utils import *
import numpy as np
from collections import namedtuple
import networkx as nx
import matplotlib.pyplot as plt
from time import sleep


# def map_data_extract(init_map_data):
#         lines = init_map_data.split('\n')
#         first_line = lines[0].split(',')

#         obstacles = []
#         available_points = []

#         map_x = int(first_line[1])
#         map_y = int(first_line[0])
#         map_size = [map_x, map_y]

#         while("" in lines):
#             lines.remove("")

#         yi = map_y
#         for line in lines[1:]:
#             l = line.split(",")
            
#             for xi in range(len(l)):
#                 if l[xi] == "1":
#                     obstacles.append((xi+1, yi))
#                 elif l[xi] == "0":
#                     available_points.append((xi+1, yi))            
#             yi -= 1 # from up to down
#         return map_size, obstacles, available_points

##########################################
### Definition of classes
##########################################

# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
#         self.obstacle = False
    
#     def up(self):
#         self.x = self.x
#         self.y += 1     
    
#     def down(self):
#         self.x = self.x
#         self.y -= 1

#     def left(self):
#         self.x -= 1
#         self.y = self.y

#     def right(self):
#         self.x += 1
#         self.y = self.y

#     def __is_obstacle(self):
#         self.obstacle = True

'''
To reduce complexity of code, class Point is no longer in use. Points are represented by tuples (x, y) instead, where x, y in range(0, size-1).
'''

class Map:
    def __init__(self):
        self.map_size = []
        self.obstacles = []
        self.available_points = []
        self.map_graph = nx.Graph() # nx graph corresponding to InitMap.txt and following graph-based path finding computation
        self.y_reversed_map_graph = nx.Graph() # nx graph corresponding to RobotSimulator.exe, with y_reversed by InitMap.txt

    def readInitMap(self):
        try:
            map_file = open("./../InitMap.txt", 'r')
            print("Loading map file to Map...")
            map_data = map_file.read()
            lines = map_data.split('\n')
            first_line = lines[0].split(',')

            map_x = int(first_line[1])
            map_y = int(first_line[0])

            self.map_size = [map_x, map_y]
            while("" in lines):
                lines.remove("")
            
            yi = map_y - 1
            for line in lines[1:]:
                l = line.split(",")

                for xi in range(len(l)):
                    if l[xi] == "1":
                        self.obstacles.append((xi, yi))
                    elif l[xi] == "0":
                        self.available_points.append((xi, yi))
                yi -= 1
            # self.map_size, self.obstacles, self.available_points = map_data_extract(map_data)
            # print(init_map)
            self.obstacles.sort(key=lambda x: x[1])
            self.obstacles.sort(key=lambda x: x[0])
            map_file.close()
            print("Map file loading complete...")
        except IOError:
            print("Initial map file not found...")

    def getMapTopologyMatrix(self):
        map_x, map_y = self.map_size[1], self.map_size[0]
        
        self.map_graph = nx.grid_2d_graph(map_x, map_y)
        self.y_reversed_map_graph = nx.grid_2d_graph(map_x, map_y)

        self.map_graph.pos = dict((n ,n) for n in self.map_graph.nodes())
        self.y_reversed_map_graph.pos = dict((n ,n) for n in self.y_reversed_map_graph.nodes())

        obs = [ob for ob in self.obstacles]
        obs_y_reversed = [(ob[0], map_y - 1 - ob[1]) for ob in self.obstacles]

        self.map_graph.remove_nodes_from(obs)
        self.y_reversed_map_graph.remove_nodes_from(obs_y_reversed)
        print("Getting map topology data ready...")
        
    def plotMapGraph(self, reversed=False):
        if reversed:
            nx.draw_networkx(self.y_reversed_map_graph, self.y_reversed_map_graph.pos, with_labels=False, node_size=0)
        else:
            nx.draw_networkx(self.map_graph, self.map_graph.pos, with_labels=False, node_size=0)
        plt.show()

    def getMapReady4System(self, map_topology_preview=False):
        if map_topology_preview:
            self.readInitMap()
            self.getMapTopologyMatrix()
            self.plotMapGraph(reversed=True)
            print("Map file info is ready for Multi-robot system")
        else:
            self.readInitMap()
            self.getMapTopologyMatrix()
            print("Map file info is ready for Multi-robot system")

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

    def readTaskfromFile(self):
        try:
            task_file = open("./../Task.txt", 'r')
            task_list = task_file.read()
            lines = task_list.split("\n")

            first_line = int(lines[0])
            self.total_number_of_tasks_in_pool = first_line     

            while ("" in lines):
                lines.remove("")
            
            for line in lines[1:]:
                # self.task_list_info.append(line)
                line = line.split(" ")
                task_i = []
                for l in line:
                    task_i.append(int(l))
                
                self.task_list_info.append(task_i)
            task_file.close()
            self.resetFlag = False
            print("Task file is loaded to task pool...")
        except IOError:
            print("Task file not found...")
        
    def taskGenerator(self, task_generation_mode="sequential"):
        if (self.resetFlag == True):
            return print("No tasks available, please read tasks from file...\n")
        
        else:
            if (task_generation_mode == "sequential"):
                try:
                    self.task_id += 1
                    
                    if (self.task_id <= self.total_number_of_tasks_in_pool):

                        task_item = self.task_list_info[self.task_id]
                        self.publish_time = task_item[1]
                        self.waiting_time = task_item[2]

                        self.starting_position = (task_item[3], task_item[4])
                        self.target_position = (task_item[5], task_item[6])
                        self.total_number_of_tasks_assigned += 1
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

    def artificialTaskGenerator(self, map_x, map_y, top2bot=None):
        # generate tasks from top to bottom (or reversed)
        pass

    def taskReset(self):
        self.task_id = 0
        self.publish_time = []
        self.waiting_time = []
        self.total_number_of_tasks_in_pool_in_pool = 0
        # self.task_list_info = []
        self.starting_position = (0, 0)
        self.target_position = (0, 0)


class Strategy:
    def __init__(self, method="default"):
        self.method = method

    # Floyd-Warshall Algorithm
    def default(self):
        pass

    def AStar(self):
        pass

    def minorityGame(self):
        pass

    def miscellaneous(self):
        pass


class Robot:
    def __init__(self, robot_index, imap):
        print("Creating and initializing Robot number ", robot_index)
        self.idx = robot_index # index starts from 0
        self.robot_local_map = imap.map_graph # nx.Graph(), 

        print("Calculating local map topological info...")
        self.predecessors, self.distance = nx.floyd_warshall_predecessor_and_distance(self.robot_local_map)
        print("Initial map info loaded...")

        self.initial_position = (0, 0)
        self.current_position = (0, 0)
        self.target_position = (0, 0)
        self.next_position = (0, 0)

        self.conflict_flag = False
        self.conflict_position = (0, 0)

        self.status = "init" # 1.init, 2.on duty, 3.task completed 4.waiting for cmd
        self.strategy = Strategy("default")
        self.priority_ranking = self.idx # initial priority ranking, update when one task is complete

        self.predicted_path = [] # a list of Points from init_pos to tar_pos
        self.optimal_path = "yes"
        self.all_available_paths = {} # all 5 actions (up, down, left, right, suspension)
        self.suspension_records = {} # take down (suspension_time, position) during recent tasks 
        
        # for smart robots
        # self.trajectory_history = [] # (Point, status, global_runtime)
        self.suspension_time_map = nx.Graph() # congestion map

    # no conflict happens, use naive path generator
    def naivePathGenerator(self):
        self.predicted_path = nx.reconstruct_path(self.current_position, self.target_position, self.predecessors)

        # naive FW path
        if (len(self.predicted_path) > 1):
            next_position = self.predicted_path[1]

            self.next_position = self.predicted_path[1]
            
            dist = self.distance[self.current_position][self.target_position]
            self.all_available_paths.update({next_position: [self.predicted_path, dist]})
        # FW path under suspension 
        else:
            self.next_position = self.current_position
            # add one node in predicted path
            # update suspension record

    # compute and check all available paths when conflict happens 
    def suboptimalPathGenerator(self):
        # generate 3 (maximum) suboptimal path using Floyd-Warshall algorithm
        subopt_paths = {}
        # neighbor represents all possible positions for next_step
        for neighbor in self.robot_local_map.neighbors(self.current_position):
            dist = self.distance[self.current_position][neighbor] + self.distance[neighbor][self.target_position]

            path = nx.reconstruct_path(neighbor, self.target_position, self.predecessors)
            path.insert(0, self.current_position)

            subopt_paths.update({neighbor: [path, dist]})

        return sorted(subopt_paths.items(), key=lambda x: x[1][1])


    # generate all paths (including optimal, suboptimal and suspension paths) and assign to self.all_available_paths dictionary
    def adaptivePathGenerator(self):
        self.all_available_paths = {}
        subopt = self.suboptimalPathGenerator()
        for item in subopt:
            self.all_available_paths.update({item[0]: [item[1][0], item[1][1]]})
        # print(self.all_available_paths)

        # add suspension as default path
        all_available_path_items = self.all_available_paths.items()
        first_item = list(all_available_path_items)[0]
        suspension_next_position = self.current_position

        first_path = first_item[1][0].copy() # copy a list, rather than assign a reference
        first_path.insert(0, self.current_position)

        suspension_path = first_path
        suspension_dist = first_item[1][1] + 1 # one suspension is equivalent to adding 1 to distance in path 

        self.all_available_paths.update({suspension_next_position: [suspension_path, suspension_dist]})
    

    
    def adaptivePathSelector(self, exempted_next_positions):
        self.adaptivePathGenerator()

        for item in self.all_available_paths.items():
            if (exempted_next_positions.count(item[0])):
                continue
            else:
                self.predicted_path = item[1][0] # assign subopt path to predicted path
                self.next_position = self.predicted_path[1]
                self.optimal_path = "no"
                break



    def statusCheck(self):
        if (self.current_position == self.target_position):
            self.status = "task_complete"
            self.weightMapUpdate() # update weight map
            # halt at target position at the moment
            # re-assign new tasks in later versions
            # update priority ranking

        elif (self.current_position == self.initial_position):
            self.status = "init"
        elif (self.current_position == self.next_position):
            self.status = "suspension"
            self.appendSuspensionTime2Map()

        else:
            self.status = "on_duty"
    

    # update weight map upon historical road experience (suspension time)
    def weightMapUpdate(self):
        if (self.strategy.method == "default"):
            pass
        else:
            # update weight map upon congestion map once every task is completed
            pass
    
    def appendSuspensionTime2Map(self):
        pass

    def executeAction(self):
        try:
            self.current_position = self.predicted_path[1]
            self.predicted_path.pop(0)
        except IndexError:
            # stay at current position
            #self.current_position = self.predicted_path[0]
            pass
        self.statusCheck() # update status
        
    
class Record:
    def __init__(self, robots, imap_graph):
        # robots: a list of Robot
        # imap_graph: nx.Graph()
        self.global_runtime = 0
        self.number_of_robots = len(robots)
        self.map_info = imap_graph

        self.current_step_info = []
        self.next_step_info = []
        self.record_book = {}
        self.conflict_robots = {}

        self.planned_next_step_conflict_flag = False

    def takeCurrentStepRecord(self, robots):
        self.current_step_info = []
        rbt_log = namedtuple('rbt_log', ['idx', 'current_pos', 'next_pos', 'target_pos'])
        for robot in robots:
            rbt = rbt_log(robot.idx, robot.current_position, robot.next_position, robot.target_position)
            self.current_step_info.append(rbt)
        self.record_book.update({self.global_runtime: self.current_step_info})


    def conflictCheck(self):
        rbt_log_list = list(self.record_book.items())[0][1] # a list of rbt_log data
        next_conflict_dict, current_position_dict = findConflictDictInfo(rbt_log_list)

        # check conflicts
        # # # # # # # 
        if (any((len(item[1]) > 1) for item in next_conflict_dict.items())): # two or more robots will take the same position in next step
            self.planned_next_step_conflict_flag = True
            # print("Conflict happens... Actions required")
            self.conflict_robots = [next_conflict_dict, current_position_dict]
            # return print(conflict_dict)
        else: # no next step conflicts
            # next takes non-self current check
            for item in next_conflict_dict.items():
                next_pos = item[0]
                idx = item[1][0]
                try:
                    if (current_position_dict[next_pos] != idx): # no current takes this next pos
                        # print("Planned pos takes other robot's current pos")
                        # print("robot " + str(idx) + " next_pos" + str(next_pos) + " takes" + " robot " + str(current_position_dict[next_pos]) + " current pos")
                        self.planned_next_step_conflict_flag = True
                        # print("Conflict happens... Actions required")
                        self.conflict_robots = [next_conflict_dict, current_position_dict]
                        break
                except KeyError:
                    pass
            else:
                print("Conflict-free paths found")
                self.planned_next_step_conflict_flag = False
                self.conflict_robots = [[], []]

    def push2Display(self):
        pos_text = str(self.number_of_robots) + "\n"

        for Rbt_log, i in zip(self.current_step_info, range(self.number_of_robots)):
            pos_text = pos_text + str(Rbt_log.idx + 1) + "," + str(Rbt_log.current_pos[0] + 1) + "," + str(Rbt_log.current_pos[1]+ 1) + "," + str(Rbt_log.target_pos[0]+ 1) + "," + str(Rbt_log.target_pos[0]+ 1) + "\n"

        with open("./../Robot_Current_Position.txt", "w") as f:
            f.write(pos_text)
        f.close()

    def takeValidRecord(self):
        self.record_book[self.global_runtime] = self.current_step_info
        self.global_runtime += 1

    def writeRecordBook2File(self):
        pass


class System:
    def __init__(self, task, imap):
        # initialized from map file
        self.map = imap
        self.map_graph = imap.map_graph

        # initialized from initial position file 
        self.total_number_of_robots = 0
        self.robots = []

        # read from file (indexed from 1), use once only
        self.initial_positions = {} # dict, use once only

        self.task = task
        self.total_run_time = 1000 # 1000 seconds (steps)
        
        # a diary to capture movements of robots
        self.Global_time_step = 0
        self.record = Record(self.robots, self.map_graph)

    def readInitPositionsfromFile(self):
        try:
            with open("./../Robot_Init_Position.txt", "r") as init_pos_file:
                print("Initial position file is loaded to System")
                init_pos_data = init_pos_file.read()
                first_line = int(init_pos_data[0])
                
                init_pos_list = []
                # self.total_number_of_robots = first_line
                lines = init_pos_data.split("\n")
                for line in lines[1:]:
                    line = line.split(",")
                    ln = []
                    for l in line:
                        ln.append(int(l))
                    init_pos_list.append(ln)

                # add initial position info to list
                for i in range(len(init_pos_list)):
                    self.initial_positions.update({i: (init_pos_list[i][1], init_pos_list[i][2])})
            init_pos_file.close()

        except IOError:
            print("Initial position file not found...")
    
    def createRobotsAtInitialPositions(self):
        self.total_number_of_robots = len(self.initial_positions)
        print("Creating robots in the MAS system...\n")
        print("This may take a while, please wait...")
        self.robots = [Robot(rbt, self.map) for rbt in range(self.total_number_of_robots)]

        for robot, i in zip(self.robots, range(len(self.initial_positions))):
            robot.initial_position = OneIdx2ZeroIdx(self.initial_positions[i])
            robot.current_position = OneIdx2ZeroIdx(self.initial_positions[i])
            # print(OneIdx2ZeroIdx(self.initial_positions[i]))
        print("All robots are deployed at initial positions")
    # use as initial task generator, later tasks are generated independently by individual robots.   
    def publishFirstTasks4AllRobots(self, task_generation_mode="sequential"):
        for robot in self.robots:
            self.task.taskGenerator(task_generation_mode) # generate tasks from the loaded task list   
            robot.target_position = self.task.target_position
        
        self.record = Record(self.robots, self.map_graph)
        self.push2Display()
        sleep(2)
        

    # pf generation function moved to class Robot
    def pfSolutionGenerator(self):
        # a trivial way to generate predicted path for each robot
        # more descent strategies to be appended later
        '''
        for robot in self.robots:
            dx, dy = (robot.target_position.x - robot.current_position.x), (robot.target_position.y - robot.current_position.y)
            robot.predicted_path.append(robot.current_position)

            if dx == 0:
                pass
            else:
                # increment x, hold y 
                for x_move in range(np.sign(dx), dx+np.sign(dx), np.sign(dx)):
                    robot.predicted_path.append(Point(robot.current_position.x + x_move, robot.current_position.y))
    
            if dy == 0:
                pass
            else:
                # increment y, hold x
                for y_move in range(np.sign(dy), dy+np.sign(dy), np.sign(dy)):
                    robot.predicted_path.append(Point(robot.current_position.x + dx, robot.current_position.y + y_move))
        '''
        # for robot in self.robots:
        #     print(robot.idx)
        #     print("Current position: ", robot.current_position.x, robot.current_position.y)
        #     print("Target position: ", robot.target_position.x, robot.target_position.y)

        #     print("A trivial solution path")
        #     for path in robot.predicted_path:
        #         print(path.x, path.y)

        # # Floyd-Warshall PF algorithm
        # path reconstruct
        # my_path = []
        # for robot in self.robots:
        #     my_path = nx.reconstruct_path(Point2double_element_list(robot.current_position), Point2double_element_list(robot.target_position), self.predecessors)
            
        #     # print(my_path)
        #     for pt in my_path:
        #         robot.predicted_path.append(Point(pt[0] + 1, pt[1] + 1))
        pass
    
    def initialPathGeneration4AllRobots(self):
        self.record = Record(self.robots, self.map_graph)
        
        for robot in self.robots:
            robot.naivePathGenerator()
        
        # take down records and check feasibility for initial naive paths
        self.record.takeCurrentStepRecord(self.robots)
        self.record.conflictCheck()

        # resolve all conflicts
        while (self.record.planned_next_step_conflict_flag):
            self.conflictResolve()
            self.record.takeCurrentStepRecord(self.robots)
            self.record.conflictCheck()

        # print("Feasible paths found!")

    
    def conflictResolve(self):
        # tell robots with conflicts to replan their paths
        robot_indices_path_replanned = []
        exempted_positions = [] # positions taken by other robots in the next step and current positions
        # max_conflict_number = 1

        next_conflict_dict = self.record.conflict_robots[0]

        current_position_dict = self.record.conflict_robots[1]



        # highest priority ranking robots
        for item in next_conflict_dict.items():
            sorted_local_ranking_indices = sorted(item[1]) # max 4
            if (len(sorted_local_ranking_indices) > 1):
                # find all robots that need to replan their paths
                robot_indices_path_replanned.append(sorted_local_ranking_indices[1:])
            
            # add all captured positions (next step)
            exempted_positions.append(item[0])
        # add current caputured positions to exemption list
        for item in current_position_dict.items():
            exempted_positions.append(item[0])
        
        
        # resolve next pos conflicts
        max_depth = findMaxDepth(robot_indices_path_replanned)
        for elimination_iter in range(0, max_depth):
            for inner_robot_list in robot_indices_path_replanned:
                idx = inner_robot_list[0]
                # print(idx)

                # 2020-10-15 2:48 pm
                # find available suboptimal path for self.robot[idx]
                exempted_positions_by_idx = []
                exempted_positions_by_idx = exempted_positions.copy()
                while (any(pos == self.robots[idx].current_position for pos in exempted_positions_by_idx)):
                    exempted_positions_by_idx.remove(self.robots[idx].current_position)
                
                self.robots[idx].adaptivePathSelector(exempted_positions_by_idx)

                exempted_positions.append(self.robots[idx].next_position)

                inner_robot_list.pop(0)
            
            while([] in robot_indices_path_replanned):
                robot_indices_path_replanned.remove([])

        # resolve next -> cur conflicts


        for robot in self.robots:
            try:
                index = current_position_dict[robot.next_position]
                if (index == robot.idx):
                    pass
                else:
                    robot.adaptivePathSelector(exempted_positions)
                    exempted_positions.append(robot.next_position)

            except KeyError: # next pos of robot does not conflicts with current robots
                pass



    # System.run() for the first step
    def runFirstStepTest(self):
        # 1. load mat.txt file & load init_pos.txt file 
        self.readInitPositionsfromFile()
        self.createRobotsAtInitialPositions()
        print("MAS is created...")
        # 2. generate init tasks for all robots
        self.publishNewTasks4all(self.task, task_generation_mode="sequential")
        print("New tasks are generated for all robots")
        # 3. each robot in self.robots computes init optimal path by using FW algorithm
        for robot in self.robots:
            robot.predecessors, robot.distance = self.predecessors, self.distance
            robot.naivePathGenerator()
        print("Initial paths generation completed")
        # 4. record, next step conflict check

        # 5. execute feasible actions
        for robot in self.robots:
            robot.executeAction()
        # 6. record, write to file, display on UI 

    # 
    def runOnce(self, step):
        # # self.record = Record(self.robots, self.map_graph)
        # self.initialPathGeneration4AllRobots()
        # self.record.takeCurrentStepRecord(self.robots)

        # # conflict check & resolve
        # while (self.conflictCheck4NextStep()):
        #     print("Collision happens in next step. Trajectory is replanning...")
        #     self.conflictResolve()

        # for robot in self.robots:
        #     robot.executeAction()
        # self.write2PositionFile()
        # self.Global_time_step += 1

        # # robot status check, assign new tasks to free robots
        # print("Global step:", self.Global_time_step)
        
        # check this 2020-10-15 ctd
        for s in range(step):
            self.takeAction()

            # self.record.takeCurrentStepRecord(self.robots)
            self.push2Display()
        
            # take down records and check feasibility for initial naive paths
            for robot in self.robots:
                robot.naivePathGenerator()
            
            self.record.takeCurrentStepRecord(self.robots)
            self.record.conflictCheck()

            # resolve all conflicts
            while (self.record.planned_next_step_conflict_flag):
                self.conflictResolve()
                self.record.takeCurrentStepRecord(self.robots)
                self.record.conflictCheck()
                print("Solving local conflicts...")

            sleep(1)

    
    # or def in class Robot
    def robotStatusCheck(self):
        # for robot in self.robots:
        #     if (robot.status == "task_complete"):
        #         # halt or re-assign new tasks
        #     pass
        pass

    def takeAction(self):
        for robot in self.robots:
                robot.executeAction()


    # # write Robot_Current_Position.txt, move this function to class Record
    def push2Display(self):
        pos_text = str(self.total_number_of_robots) + "\n"
        
        for robot, i in zip(self.robots, range(self.total_number_of_robots)):
            pos_text = pos_text + str(robot.idx + 1) + "," + str(robot.current_position[0] + 1) + "," + str(robot.current_position[1] + 1) + "," + str(robot.target_position[0] + 1) + "," + str(robot.target_position[1] + 1) + "\n"
        

        with open("./../Robot_Current_Position.txt", "w") as f:
            f.write(pos_text)
        f.close()
    #     # writeRecords("Step" + "\n" + pos_text + "\n")
