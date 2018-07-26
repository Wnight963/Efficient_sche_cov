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
f = open('store.pckl', 'rb')
robots = pickle.load(f)
f.close()
# these robots can configured completing the first task


print(robots)
for x in robots:
    print(x.number, x.role, x.location)
