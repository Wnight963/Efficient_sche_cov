from robot_class import ROBOT
from robot_class import leader_election
from robot_class import leader_move
from robot_class import recruit_election

import numpy as np
N = 11    # number of robots
K = 1     # number of base station


target = np.array([1.6, 1.6])
# location is represented by np.array

initial_location = np.random.uniform(0, 0.2, size=[N+K,2])

robots = [ROBOT(i, initial_location[i]) for i in range(N+K)]

############################################### leader election, leader moves and get stucked
############################################### in local stationary
robots, leader_index = leader_election(robots, target)
robots = leader_move(robots, leader_index, target)

############################################### recruit election,

# from robot_class import location_extraction
# x = location_extraction(robots)
# import matplotlib.pyplot as plt
# plt.figure("final position")
# plt.scatter(x[:,0],x[:,1])
# n = range(N+K)
# for i,txt in enumerate(n):
#     plt.annotate(txt, (x[i,0],x[i,1]))
# plt.show()

###############################################
