from robot_class import location_extraction
from robot_class import routing_strategy_extraction
from robot_class import single_node_move

import numpy.linalg as LA
import sys

sys.path.append(r'../communication_model')
from commu_model import optimal_routing

N = 11
K = 1

def active_team_move(robot_list, moving_robot_index, target):
    'this function updates locations of robots in active leader team'
    'moving robot index is an orded set, the first one is nearest to the AP,'
    'the last one is leader'
    sigma = 0.1
    location = location_extraction(robot_list)
    new_location = location
    team_length = len(moving_robot_index)
    routing_strategy = routing_strategy_extraction(robot_list)

    while(1):
        location = new_location
        for i in range(team_length):
            if(i<team_length-1):
                new_location[moving_robot_index[i]] = single_node_move(robot_list, moving_robot_index[i],
                                     robot_list[moving_robot_index[i+1]].location, routing_strategy, sigma)
            else:
                print("I am executed")
                new_location[moving_robot_index[i]] = single_node_move(robot_list, moving_robot_index[i],
                                     target, routing_strategy, sigma)
        if(max(LA.norm((new_location-location), axis=1))<=0.01):
            break
        unuse, routing_strategy = optimal_routing(new_location)
    for i,x in enumerate(robot_list):
        if(i<N):
            x.location_update(new_location[i])
            x.routing_strategy_update(routing_strategy[i])
    return robot_list
