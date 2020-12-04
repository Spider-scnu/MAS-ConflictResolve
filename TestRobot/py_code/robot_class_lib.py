import networkx as nx
from matplotlib.pyplot import show
from random import uniform

class Robot:
    def __init__(self, robot_index, imap):
        print("Creating Robot No. ", robot_index)
        self.idx = robot_index # index starts from 0
        self.robot_local_map = imap.y_reversed_map_graph # nx.Graph()

        if ((not imap.predecessors) & (not imap.distance)):
            self.FloydWarshall = False
            self.predecessors, self.distance = [], []
        else:
            print("Geting FW matrices from Map...")
            self.predecessors, self.distance = imap.predecessors, imap.distance

        self.initial_position = (0, 0)
        self.current_position = (0, 0)
        self.target_position = (0, 0)
        self.next_position = (0, 0)
        self.status = "init" # 1.init, 2.on duty, 3.task completed 4.waiting for cmd

        self.conflict_flag = False
        self.conflict_position = (0, 0)

        # self.strategy = strategy
        self.priority_ranking = self.idx # initial priority ranking, update when one task is complete

        self.predicted_path = [] # a list of Points from init_pos to tar_pos
        self.optimal_path = "tba"
        self.all_available_paths = {} # all 5 actions (up, down, left, right, suspension)
        
        # for smart robots
        # self.trajectory_history = [] # (Point, status, global_runtime)
        # self.suspension_time_map = nx.Graph() # congestion map
        # self.suspension_records = {} # take down (suspension_time, position) during recent tasks 

    # no conflict happens, use naive path generator
    def naivePathGenerator(self, algorithm="AStar"):
        if (algorithm == "FloydWarshall"):
            self.naivePathGenerator_FW()
        elif (algorithm == "AStar"):
            self.naivePathGenerator_AStar()
        else:
            print("Please select a path planning algorithm")
            raise ValueError

    def naivePathGenerator_FW(self):
        if (self.predecessors == []):
            print("Floyd-Warshall matrices not found...")
            raise ValueError
        else:
            self.predicted_path = nx.reconstruct_path(self.current_position, self.target_position, self.predecessors)

            # naive FW path
            if (len(self.predicted_path) > 1):
                next_position = self.predicted_path[1]
                self.next_position = self.predicted_path[1]
                dist = self.distance[self.current_position][self.target_position]

                self.all_available_paths.update({next_position: [self.predicted_path, dist]})
                # print(dist)
            else:
                self.predicted_path = [self.current_position]
                self.next_position = self.current_position
                dist = self.distance[self.current_position][self.target_position]

                self.all_available_paths.update({self.next_position: [self.predicted_path, dist]})
                # add one node in predicted path
                # update suspension record
        
    def naivePathGenerator_AStar(self):
        # self.predicted_path = nx.astar_path(self.robot_local_map, self.current_position, self.target_position)
        # dist = nx.astar_path_length(self.robot_local_map, self.current_position, self.target_position)
        self.predicted_path = nx.shortest_path(self.robot_local_map, self.current_position, self.target_position)
        dist = nx.shortest_path_length(self.robot_local_map, self.current_position, self.target_position)
        # print(dist)
        if (len(self.predicted_path) >= 2):
                next_position = self.predicted_path[1]
                self.next_position = self.predicted_path[1]
                self.all_available_paths.update({next_position: [self.predicted_path, dist]})
        else:
            self.predicted_path = [self.current_position]
            self.next_position = self.current_position
            self.all_available_paths.update({self.next_position: [self.predicted_path, dist]})

    # compute and check all available paths when conflict happens 
    def suboptimalPathGenerator(self, algorithm="AStar"):
        if (algorithm == "FloydWarshall"):
            return self.suboptimalPathGenerator_FW()
        elif (algorithm == "AStar"):
            return self.suboptimalPathGenerator_AStar()
        else:
            print("Please select a path planning algorithm")
            raise ValueError

    def suboptimalPathGenerator_FW(self):
        subopt_paths = {}
        if (self.current_position == self.target_position):
            subopt_paths.update({self.target_position: [[self.target_position], 0]})
            return sorted(subopt_paths.items(), key=lambda x: x[1][1])
        else:
            for neighbor in self.robot_local_map.neighbors(self.current_position):
                dist = self.distance[self.current_position][neighbor] + self.distance[neighbor][self.target_position]
                path = nx.reconstruct_path(neighbor, self.target_position, self.predecessors)
                path.insert(0, self.current_position)

                subopt_paths.update({neighbor: [path, dist]})
        return sorted(subopt_paths.items(), key=lambda x: x[1][1])

    def suboptimalPathGenerator_AStar(self):
        subopt_paths = {}
        if (self.current_position == self.target_position):
            subopt_paths.update({self.target_position: [[self.target_position], 0]})
            return subopt_paths
        else:
            for neighbor in self.robot_local_map.neighbors(self.current_position):
                dist = nx.astar_path_length(self.robot_local_map, self.current_position, neighbor) + nx.astar_path_length(self.robot_local_map, neighbor, self.target_position)

                path = nx.astar_path(self.robot_local_map, neighbor, self.target_position)
                path.insert(0, self.current_position)

                subopt_paths.update({neighbor: [path, dist]})
        return sorted(subopt_paths.items(), key=lambda x: x[1][1])

    def adaptivePathGenerator(self, algorithm="AStar"):
        self.all_available_paths = {}
        subopt = self.suboptimalPathGenerator(algorithm=algorithm)
        if (self.current_position == self.target_position):
            min_dist = 0
            self.all_available_paths.update({self.current_position: [[self.current_position], 0]})
        else:
            for item in subopt:
                self.all_available_paths.update({item[0]: [item[1][0], item[1][1]]})

            best = min(self.all_available_paths.items(), key=lambda x:x[1][1])
            min_dist = best[1][1]
            min_path = best[1][0]
            suspension_path = min_path.copy()
            suspension_path.insert(0, self.current_position)
            # add suspension/robot stays at current position
            self.all_available_paths.update({self.current_position: [suspension_path, min_dist+1]})

            if (uniform(0, 1) * self.idx <= 7):
                if (uniform(0, 1) <= 0.2):
                    sorted_paths = sorted(self.all_available_paths.items(), key=lambda x: x[1][1], reverse=True)
                else:
                    sorted_paths = sorted(self.all_available_paths.items(), key=lambda x: x[1][1])

                self.all_available_paths = {}
                for item in sorted_paths:
                    self.all_available_paths.update({item[0]: [item[1][0], item[1][1]]})

        # return self.all_available_paths

    def adaptivePathSelector(self, exempted_next_positions, algorithm="AStar"):
        # generate all available paths
        self.adaptivePathGenerator(algorithm=algorithm)

        if (self.current_position == self.target_position):
            self.predicted_path = [self.current_position]
            self.next_position = self.current_position
            return

        # exempted_next_postion list is empty, no neighbor nodes captured
        if (not exempted_next_positions):
            self.naivePathGenerator()
            print("No neighbor nodes are captured")
            return

        for item in self.all_available_paths.items():
            if (exempted_next_positions.count(item[0])):
                if (item[0] == self.current_position):
                    # suspension path is selected
                    self.predicted_path = item[1][0]
                    self.next_position = self.current_position
                    return
                else:
                    pass
                # check next optional path is valid or not
                continue
            else:
                self.predicted_path = item[1][0] # assign subopt path to predicted path
                self.next_position = self.predicted_path[1]
                # self.optimal_path = "no"
                return

    def statusCheck(self):
        if (self.current_position == self.target_position):
            self.status = "task_complete"
            # self.weightMapUpdate() # update weight map
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
        pass
    
    def appendSuspensionTime2Map(self):
        pass

    def executeAction(self):
        try:
            self.current_position = self.predicted_path[1]
            self.next_position = self.predicted_path[2]
            self.predicted_path.pop(0)
        except IndexError:
            # stay at current position
            #self.current_position = self.predicted_path[0]
            pass
        self.statusCheck() # update status

    def plotPath(self):
        nodes = self.predicted_path
        pos = dict((n,n) for n in self.robot_local_map.nodes())
        nx.draw_networkx(self.robot_local_map, pos, with_labels=False, node_size=0)
        nx.draw_networkx_nodes(self.robot_local_map, pos, nodelist=nodes, node_size=5)
        show()

    def push2Display(self):
        pass

    def addNodes(self, node_list):
        self.robot_local_map.add_nodes_from(node_list)

    def plotLocalMap(self):
        pos = dict((n,n) for n in self.robot_local_map.nodes())
        nx.draw_networkx(self.robot_local_map, pos)
        show()