# 2st task which does needs secondary leader team

from robot_class import ROBOT
from robot_class import leader_election
from robot_class import leader_move
from robot_class import recruit_election
from robot_class import secondary_leader_team_construction
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



for x in robots:
    x.pre_role = "redundant_node"
    print(x.number, x.role, x.pre_role, x.location)
from routing_present import routing_graph_for_robot_list
routing_graph_for_robot_list(robots)
# graph visualized


target2 = np.array([4.8, 1.2])


# import networkx as nx
# import matplotlib.pyplot as plt
# from robot_class import channel_matrix_extraction
# from robot_class import routing_strategy_extraction
#
# G = nx.DiGraph()
# G.add_nodes_from(range(N+K))
# T = routing_strategy_extraction(robots)
# R = channel_matrix_extraction(robots)
# transmission_data = T * R
# edges = [(j,i) for i in range(N) for j in range(N+K) if transmission_data[i,j]>0]
# G.add_edges_from(edges)
# shortest_path = nx.shortest_path(G, source=11, target=3)
# print(shortest_path)
# print([robots[x].role for x in shortest_path])
# nx.draw(G, with_labels=True, font_weight='bold')
# plt.show()
robots, leader_index = leader_election(robots, target2)
robots, recruit_index = recruit_election(robots)
if(robots[leader_index].pre_role=='leaf'):

    secondary_leader_team_index = secondary_leader_team_construction(
        robots, recruit_index, leader_index, after_leader_election=True)
    print(secondary_leader_team_index)
    ########### once the leader is elected and it's previous role is a leaf node
    ########### a recruit election is immediately induced to release the leader from
    ########### its previous location
    for i in range(len(secondary_leader_team_index)-1):
        robots = active_team_move(robots, secondary_leader_team_index[i],
                                  robots[secondary_leader_team_index[i+1][0]].location)
        if(i<len(secondary_leader_team_index)-2):
            tmp_role = robots[secondary_leader_team_index[i+1][0]].role
        else:
            tmp_role = robots[leader_index].pre_role
        robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
    
    ###########
    primary_leader_team = [leader_index]
    ########## primary team now has only one member leader
    robots = active_team_move(robots, primary_leader_team, target2)
    ########## primary team moves and gets trapped in local stationary point
    while(LA.norm(robots[leader_index].location-target2)>=0.2):
        robots, recruit_index = recruit_election(robots)
        secondary_leader_team_index = secondary_leader_team_construction(
            robots, recruit_index, primary_leader_team[0], after_leader_election=False)
        print(secondary_leader_team_index)
        for i in range(len(secondary_leader_team_index) - 1):
            robots = active_team_move(robots, secondary_leader_team_index[i],
                                      robots[secondary_leader_team_index[i + 1][0]].location)
            if (i < len(secondary_leader_team_index) - 2):
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
            else:
                tmp_role = robots[leader_index].pre_role
            robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
        primary_leader_team.insert(0, secondary_leader_team_index[-1][0])
        robots[primary_leader_team[0]].role_update("node")
        robots = active_team_move(robots, primary_leader_team, target2)
    if (LA.norm(robots[leader_index].location - target2)<0.2):
        print("task completed!")
    for x in robots:
        if (x.role == 'leader'):
            x.role_update('leaf')
elif(robots[leader_index].pre_role == 'node' or robots[leader_index].pre_role == 'junction'):

    secondary_leader_team_index = secondary_leader_team_construction(
        robots, recruit_index, leader_index, after_leader_election=True)
    print(secondary_leader_team_index)
    ########### once the leader is elected and it's previous role is a leaf node
    ########### a recruit election is immediately induced to release the leader from
    ########### its previous location
    for i in range(len(secondary_leader_team_index) - 1):
        robots = active_team_move(robots, secondary_leader_team_index[i],
                                  robots[secondary_leader_team_index[i + 1][0]].location)
        if (i < len(secondary_leader_team_index) - 2):
            tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
        else:
            tmp_role = 'junction'
        robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)

    ###########
    primary_leader_team = [leader_index]
    ########## primary team now has only one member leader
    robots = active_team_move(robots, primary_leader_team, target2)
    ########## primary team moves and gets trapped in local stationary point
    while (LA.norm(robots[leader_index].location - target2) >= 0.2):
        robots, recruit_index = recruit_election(robots)
        secondary_leader_team_index = secondary_leader_team_construction(
            robots, recruit_index, primary_leader_team[0], after_leader_election=False)
        print(secondary_leader_team_index)
        for i in range(len(secondary_leader_team_index) - 1):
            robots = active_team_move(robots, secondary_leader_team_index[i],
                                      robots[secondary_leader_team_index[i + 1][0]].location)
            if (i < len(secondary_leader_team_index) - 2):
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
            else:
                tmp_role = robots[leader_index].pre_role
            robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
        primary_leader_team.insert(0, secondary_leader_team_index[-1][0])
        robots[primary_leader_team[0]].role_update("node")
        robots = active_team_move(robots, primary_leader_team, target2)
    if (LA.norm(robots[leader_index].location - target2) < 0.2):
        print("task completed!")
    for x in robots:
        if (x.role == 'leader'):
            x.role_update('leaf')


for x in robots:
    print(x.number, x.role, x.pre_role, x.location)

# f = open('robots_after_2st_task.pckl', 'wb')
# pickle.dump(robots, f)
# f.close()