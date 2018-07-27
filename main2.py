# 2st task which does not need secondary leader team

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


target2 = np.array([-2, -2])
robots, leader_index = leader_election(robots, target2)
robots = leader_move(robots, leader_index, target2)
moving_robot_index = [leader_index]
while(LA.norm(robots[leader_index].location-target2)>=0.2):
    robots, recruit_index = recruit_election(robots)
    print("recruit_index:")
    print(recruit_index)
    moving_robot_index.insert(0, recruit_index)
    robots = active_team_move(robots, moving_robot_index, target2)
if(LA.norm(robots[leader_index].location-target2)<0.2):
    print("task completed!")
for x in robots:
    if (x.role == 'leader'):
        x.role_update('leaf')
    # since leader reached its destination, it becomes leaf node
    print(x.number, x.role, x.location)