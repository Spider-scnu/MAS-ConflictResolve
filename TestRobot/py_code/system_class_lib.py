import networkx as nx
from time import sleep
from mas_utils import *
from record_class_lib import Record
from robot_class_lib import Robot
from copy import deepcopy
from sys import exit

class System:
    def __init__(self, task, imap):
        # initialized from map file
        self.map = imap
        # print(self.map)
        self.initial_map_graph = imap.y_reversed_map_graph.copy()
        self.map_graph = imap.y_reversed_map_graph

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

        ######################################
        #  qiukaibin
        self.feature_map = np.ones(shape = (self.map.map_size[0], self.map.map_size[1]))

        for node in self.map.map_graph.nodes():
            self.feature_map[node[1], node[0]] = 0
        print(self.feature_map)

        self.fixed_task_complete_reward = 200
        ######################################
        

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
            robot.initial_position = self.initial_positions[i]
            robot.current_position = self.initial_positions[i]
            # print(OneIdx2ZeroIdx(self.initial_positions[i]))
        print("All robots are deployed at initial positions")
    # use as initial task generator, later tasks are generated independently by individual robots.   
    def publishFirstTasks4AllRobots(self, task_generation_mode="sequential"):
        for robot in self.robots:
            self.task.taskGenerator(task_generation_mode) # generate tasks from the loaded task list
            robot.target_position = self.task.target_position
        
        self.record = Record(self.robots, self.map_graph)
        self.push2Display()
        sleep(5)
        
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

    

    def conflictResolve(self, timestep = 0):
        # tell robots with conflicts to replan their paths
        robot_indices_path_replanned = []
        ################
        # qiukaibin
        # Observation
        # 1. self.feature_map : 地图信息，固定的，size=h*w。这是一个二值矩阵，1表示有障碍物，0表示可通行的道路。
        # 2. self.feature_agent_position : 当前冲突机器人A的位置，size=h*w*n，n是该系统中机器人数量。1表示目前所在的位置，0表示未知区域。
        # 3. self.feature_other_agent_position : 与机器人A冲突的所有机器人的位置，size=h*w*n。1表示其它机器人的位置，0表示未知区域。
        # 4. self.feature_agent_target_position : 机器人A卸货的目标位置，size=h*w*n。1表示目标位置。
        # 5. self.feature_other_agent_target_position : 与机器人A冲突的所有机器人的目标位置，size=h*w*n。
        conflict_robot_indices = []

        self.feature_agent_position = np.zeros(shape = (self.map.map_size[0], self.map.map_size[1], self.total_number_of_robots)) #np.zeros_like(self.feature_map)
        self.feature_other_agent_position = np.zeros_like(self.feature_agent_position)
        self.feature_agent_target_position = np.zeros_like(self.feature_agent_position)
        self.feature_other_agent_target_position = np.zeros_like(self.feature_agent_position)
        # self.feature_agent_route = np.zeros_like(self.feature_agent_position)
        # print('------------shape----------', self.feature_agent_position.shape)

        ##################

        all_agent_position = np.zeros_like(self.feature_map)
        for robotIndex in range(self.total_number_of_robots):
            agent_position = self.robots[robotIndex].current_position
            # agent_position[0] = self.map.map_size[0] - 1 - agent_position[0]
            all_agent_position[self.map.map_size[0] - 1 - agent_position[1], agent_position[0]] = 1
            self.feature_agent_position[self.map.map_size[0] - 1 - agent_position[1], agent_position[0],  robotIndex] = 1


            agent_target_position = self.robots[robotIndex].target_position

            self.feature_agent_target_position[self.map.map_size[0] - 1 - agent_target_position[1], agent_target_position[0],  robotIndex] = 1
            

        ################
        exempted_positions = [] # positions taken by other robots in the next step and current positions

        next_conflict_dict = self.record.conflict_robots[0]
        current_position_dict = self.record.conflict_robots[1]
        
        # highest priority ranking robots
        for item in next_conflict_dict.items():
            sorted_local_ranking_indices = item[1].copy() # max 4
            
            sorted_local_ranking_indices.sort()

            if (len(sorted_local_ranking_indices) >= 2):

                ############################
                # qiukaibin
                # 冲突的机器序号
                # [[0,1], [2, 9, 10]]
                conflict_robot_indices.append(sorted_local_ranking_indices)

                # 决定获得路权的机器人以及需要让路的机器人
                fixed_robot_indices = np.random.choice(sorted_local_ranking_indices)
                # print('----fixed_robot_indices----', fixed_robot_indices, sorted_local_ranking_indices)

                sorted_local_ranking_indices.remove(fixed_robot_indices)
                # print('----pop----', sorted_local_ranking_indices)
                #############################

                # find all robots that need to replan their paths
                robot_indices_path_replanned.append(sorted_local_ranking_indices)
            
            # add all captured positions (next step)
            exempted_positions.append(item[0])
            
        ################################
        # qiukaibin
        feature_inputs = []
        for robotPair in conflict_robot_indices:
            agent_inputs = []
            for conflictRobotIndex in robotPair:
                self.robots[conflictRobotIndex].estimate_complete_task_reward = self.fixed_task_complete_reward - timestep +\
                                                     self.robots[conflictRobotIndex].start_timestep - len(self.robots[conflictRobotIndex].predicted_path)
                # print('----estimate_complete_task_reward----', self.robots[conflictRobotIndex].estimate_complete_task_reward)
                agent_position = self.robots[conflictRobotIndex].current_position
                # agent_position[0] = self.map.map_size[0] - 1 - agent_position[0]
                self.feature_other_agent_position[:, :, conflictRobotIndex] = all_agent_position.copy()
                self.feature_other_agent_position[self.map.map_size[0] - 1 - agent_position[1], agent_position[0], conflictRobotIndex] = 0
                for otherRobotIndex in robotPair:
                    if conflictRobotIndex != otherRobotIndex:
                        other_agent_target_position = self.robots[otherRobotIndex].target_position
                        #other_agent_target_position[0] = self.map.map_size[0] - 1 - other_agent_target_position[0]
                        self.feature_other_agent_target_position[self.map.map_size[0] - 1 - other_agent_target_position[1], other_agent_target_position[0], conflictRobotIndex] = 1
                agent_input = np.zeros(shape = (self.map.map_size[0], self.map.map_size[1], 5))
                agent_input[:, :, 0] = self.feature_map.copy()
                agent_input[:, :, 1] = self.feature_agent_position[:, :, conflictRobotIndex].copy()
                agent_input[:, :, 2] = self.feature_agent_target_position[:, :, conflictRobotIndex].copy()
                agent_input[:, :, 3] = self.feature_other_agent_position[:, :, conflictRobotIndex].copy()
                agent_input[:, :, 4] = self.feature_other_agent_target_position[:, :, conflictRobotIndex].copy()
                agent_inputs.append(agent_input)
            
            feature_inputs.append(agent_inputs)
        #################################
        # add current caputured positions to exemption list
        for item in current_position_dict.items():
            exempted_positions.append(item[0])
            # print('current', item, self.robots[item[1]].current_position)
            # print()
        
        ##########################
        # qiukaibin
        #for item in next_conflict_dict.items():
        #    exempted_positions.append(item[0])
            #print('next', item)
        ##########################
        
        # resolve next pos conflicts
        max_depth = findMaxDepth(robot_indices_path_replanned)
        for elimination_iter in range(0, max_depth):
            for inner_robot_list in robot_indices_path_replanned:
                idx = inner_robot_list[0]

                self.robots[idx].adaptivePathSelector(exempted_positions, conflict_resolve = True)
                exempted_positions.append(self.robots[idx].next_position)
                # print(self.robots[idx].next_position)

                inner_robot_list.pop(0)
            
            while([] in robot_indices_path_replanned):
                robot_indices_path_replanned.remove([])


        # resolve next -> cur conflicts
        for robot in self.robots:
            # print(robot.next_position)
            try:
                index = current_position_dict[robot.next_position]
                if (index == robot.idx):
                    pass
                else:
                    robot.adaptivePathSelector(exempted_positions, conflict_resolve = True)
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

    def runOnce(self, step = 1, steptime = 0):
        for s in range(step):
            self.takeAction()
            self.push2Display()
            self.robotStatusCheck(steptime)
            self.taskCompletionCheck(steptime)
            #self.updateMap()

            self.record.takeCurrentStepRecord(self.robots)

            self.record.conflictCheck()

            # resolve all conflicts
            #print('conflict-robot', self.record.conflict_robots)
            while (self.record.planned_next_step_conflict_flag):
                #break
                self.conflictResolve(steptime)
                self.record.takeCurrentStepRecord(self.robots)
                self.record.conflictCheck()
                # print("Solving local conflicts...")
            self.updateMap(steptime)
            sleep(1)

    # or def in class Robot
    def robotStatusCheck(self, timestep = 0):
        new_obs = []
        for robot in self.robots:
            if (robot.status == "task_complete"):
                new_obs.append(robot.target_position)
                ###################
                # qiukaibin
                if (robot.task_finish == False):
                    robot.end_timestep = timestep
                    robot.account += self.fixed_task_complete_reward + robot.start_timestep - robot.end_timestep
                    print('---------robot-{} : account={}--------{}-{}'.format(robot.idx, robot.account, robot.start_timestep, robot.end_timestep))
                    robot.task_finish = True
                ###################
        return new_obs
    
    def updateMap(self, timestep = 0, exempted_positions = []):
        updated_local_map_graph = self.initial_map_graph.copy()
        new_obs = self.robotStatusCheck(timestep)
        print('----new_obs----', new_obs)
        updated_local_map_graph.remove_nodes_from(new_obs)

        for robot in self.robots:
            if (robot.status != "task_complete"):
                robot.robot_local_map = updated_local_map_graph
                #for 
            # elif (robot.status == "task_complete"):
            #     robot.robot_local_map = self.initial_map_graph
            #     self.task.taskGenerator("sequential")
            #     robot.target_position = self.task.target_position
            #     robot.naivePathGenerator()
                
        # for robot in self.robots:
        #     robot.plotPath()

    def taskCompletionCheck(self, steptime = 0):
        if (any(robot.status != "task_complete" for robot in self.robots)):
            pass
        else:
            ###########################
            # qiukaibin
            for robot in self.robots:
                robot.start_timestep = steptime
                robot.task_finish = False
            ##########################
            self.assignNewTasks4AllRobots()
            print("All tasks are completed")
            print()
            #exit()
            #self.assignNewTasks4AllRobots()

    def assignNewTasks4AllRobots(self):
        new_obs = []
        for robot in self.robots:
            if (robot.status == "task_complete"):
                new_obs.append(robot.current_position)
        
        for robot in self.robots:
            self.task.taskGenerator("sequential")
            robot.robot_local_map = self.initial_map_graph
            # updated_local_map_graph = robot.robot_local_map
            robot.addNodes(new_obs)
            #print(self.task.target_position)
            robot.target_position = self.task.target_position
            robot.status = "init"
            robot.naivePathGenerator()

    def takeAction(self):
        for robot in self.robots:
            robot.executeAction()
        
        self.record.global_runtime += 1

    # # write Robot_Current_Position.txt, move this function to class Record
    def push2Display(self):
        pos_text = str(self.total_number_of_robots) + "\n"
        
        for robot, i in zip(self.robots, range(self.total_number_of_robots)):
            # pos_text = pos_text + str(robot.idx + 1) + "," + str(robot.current_position[0] + 1) + "," + str(robot.current_position[1] + 1) + "," + str(robot.target_position[0] + 1) + "," + str(robot.target_position[1] + 1) + "\n"
            pos_text = pos_text + str(robot.idx + 1) + "," + str(robot.current_position[1] + 1) + "," + str(robot.current_position[0] + 1) + "," + str(robot.target_position[1] + 1) + "," + str(robot.target_position[0] + 1) + "\n"
        

        with open("./../Robot_Current_Position.txt", "w") as f:
            f.write(pos_text)
        f.close()
