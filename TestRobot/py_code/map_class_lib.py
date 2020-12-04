import networkx as nx
import matplotlib.pyplot as plt

class Map:
    def __init__(self):
        self.map_size = []
        self.obstacles = []
        self.available_points = []

        self.map_graph = nx.Graph() # nx graph corresponding to InitMap.txt and following graph-based path finding computation
        self.predecessors, self.distance = [], [] # for FW algorithm

        self.y_reversed_map_graph = nx.Graph() # nx graph corresponding to RobotSimulator.exe, with y_reversed by InitMap.txt

    def readInitMap(self, map_file_path="./../InitMap.txt"):
        try:
            map_file = open(map_file_path, 'r')
            print("Loading map file to Map...")
            map_data = map_file.read()
            lines = map_data.split('\n')
            first_line = lines[0].split(',')

            number_of_rows = int(first_line[0]) # hang shu
            number_of_cols = int(first_line[1]) # lie shu

            self.map_size = [number_of_rows, number_of_cols]

            while("" in lines):
                lines.remove("")
            
            yi = number_of_rows - 1
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
            self.obstacles.sort(key=lambda x: x[0])
            self.obstacles.sort(key=lambda x: x[1])
            map_file.close()
            print("Map file loading complete...")
        except IOError:
            print("Map file not found...")

    def getMapTopologyMatrix(self, FloydWarShall=False):
        map_y, map_x = self.map_size[0], self.map_size[1]
        self.map_graph = nx.grid_2d_graph(map_x, map_y)
        self.y_reversed_map_graph = nx.grid_2d_graph(map_x, map_y)

        self.map_graph.pos = dict((n,n) for n in self.map_graph.nodes())
        self.y_reversed_map_graph.pos = dict((n,n) for n in self.y_reversed_map_graph.nodes())

        # remove obstacle nodes
        obs = [ob for ob in self.obstacles]
        obs_y_reversed = [(ob[0], map_y - ob[1] - 1) for ob in self.obstacles]
        self.map_graph.remove_nodes_from(obs)
        self.y_reversed_map_graph.remove_nodes_from(obs_y_reversed)

        if (FloydWarShall == True):
            print("Calculating FW matrices...This may take a while...")
            self.predecessors, self.distance = nx.floyd_warshall_predecessor_and_distance(self.y_reversed_map_graph)
            print("Getting FW matrices ready...")
        
        print("Getting map topology data ready...")
        
    def plotMapGraph(self, reversed=False):
        if reversed:
            nx.draw_networkx(self.y_reversed_map_graph, self.y_reversed_map_graph.pos, with_labels=True, node_size=0)
        else:
            nx.draw_networkx(self.map_graph, self.map_graph.pos, with_labels=True, node_size=0)
        plt.show()

    def getMapReady4System(self, map_topology_preview=False, FW=False):
        if map_topology_preview:
            self.readInitMap()
            self.getMapTopologyMatrix(FloydWarShall=FW)
            self.plotMapGraph(reversed=True)
            print("Map file info is ready for Multi-robot system")
        else:
            self.readInitMap()
            self.getMapTopologyMatrix(FloydWarShall=FW)
            print("Map file info is ready for Multi-robot system")