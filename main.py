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

initial_location = np.random.uniform(0, 0.2, size=[N+K,2])

robots = [ROBOT(i, initial_location[i]) for i in range(N+K)]
robots[-1].role_update('AP')
############################################### leader election, leader moves and get stucked
############################################### in local stationary
robots, leader_index = leader_election(robots, target)
robots = leader_move(robots, leader_index, target)
moving_robot_index = [leader_index]
############################################### recruit election

# robots, recruit_index = recruit_election(robots)
# moving_robot_index.insert(0, recruit_index)
# robots = active_team_move(robots, moving_robot_index, target)
# robots, recruit_index = recruit_election(robots)
# moving_robot_index.insert(0, recruit_index)
# robots = active_team_move(robots, moving_robot_index, target)
# robots, recruit_index = recruit_election(robots)
# moving_robot_index.insert(0, recruit_index)
# robots = active_team_move(robots, moving_robot_index, target)


#
while(LA.norm(robots[leader_index].location-target)>=0.2):
    robots, recruit_index = recruit_election(robots)
    print("recruit_index:")
    print(recruit_index)
    moving_robot_index.insert(0, recruit_index)
    robots = active_team_move(robots, moving_robot_index, target)

if(LA.norm(robots[leader_index].location-target)<0.2):
    print("task completed!")

for x in robots:
    print(x.number, x.role, x.location)



###############################################
