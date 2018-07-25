from robot_class import location_extraction
from robot_class import single_node_move

import numpy.linalg as LA
import sys

sys.path.append(r'../communication_model')
from commu_model import optimal_routing

def active_team_move(robot_list, moving_robot_index):
    'this function updates locations of robots in active leader team'
    'moving robot index is an orded set, the first one is nearest to the AP,'
    'the last one is leader'
    sigma = 0.1
    location = location_extraction(robot_list)
    team_length = len(moving_robot_index)

    while(LA.norm(new_location - location)>=0.01):
        new_location = location
        for i in range(team_length):
            location[moving_robot_index[i]] = \
                single_node_move(robot_list, moving_robot_index[i], \
                                 moving_robot_index[i+1], routing_strategy, sigma)
        res, routing_result = optimal_routing(location, leader_index)






