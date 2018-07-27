# 2st task which does needs secondary leader team

from robot_class import ROBOT
from robot_class import leader_election
from robot_class import leader_move
from robot_class import recruit_election
import sys

sys.path.append(r'motion_model')
sys.path.append(r'communication_model')
from move_model import active_team_move

import numpy.linalg as LA
import numpy as np
N = 11    # number of robots
K = 1     # number of base station


target = np.array([5.6, 3.0])
# location is represented by np.array

import pickle
f = open('robots_after_1st_task.pckl', 'rb')
robots = pickle.load(f)
f.close()
# these robots are configured completing the first task


print(robots)
for x in robots:
    print(x.number, x.role, x.location)
from routing_present import routing_graph_for_robot_list
routing_graph_for_robot_list(robots)
# graph visualized


target2 = np.array([8, 3.2])
robots, leader_index = leader_election(robots, target)
robots, recruit_index = recruit_election(robots)
import networkx as nx
import matplotlib.pyplot as plt
from robot_class import channel_matrix_extraction
from robot_class import routing_strategy_extraction

G = nx.DiGraph()
G.add_nodes_from(range(N+K))
T = routing_strategy_extraction(robots)
R = channel_matrix_extraction(robots)
transmission_data = T * R
edges = [(j,i) for i in range(N) for j in range(N+K) if transmission_data[i,j]>0]
G.add_edges_from(edges)
shortest_path = nx.shortest_path(G, source=11, target=3)
print(shortest_path)
print([robots[x].role for x in shortest_path])
nx.draw(G, with_labels=True, font_weight='bold')
plt.show()
from robot_class import secondary_leader_team_construction
print(secondary_leader_team_construction(robots, recruit_index, leader_index, after_leader_election=True))

