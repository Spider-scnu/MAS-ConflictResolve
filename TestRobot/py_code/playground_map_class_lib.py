from map_class_lib import Map


#############################################
''' This script is for class Map testing  '''
#############################################


# create a Map class variable my_map
my_map = Map()

# read map info from ./../InitMap.txt or manually type in the dir_path
my_map.readInitMap()

# test loaded info

# print(my_map.map_size)
# print(my_map.obstacles)
# print(my_map.available_points)

# get topology
# my_map.getMapTopologyMatrix()
# # print(my_map.map_graph)
# my_map.plotMapGraph(reversed=True)

my_map.getMapReady4System(map_topology_preview=False)


import networkx as nx
import matplotlib.pyplot as plt
nx.draw_networkx(my_map.y_reversed_map_graph, my_map.y_reversed_map_graph.pos)
plt.show()