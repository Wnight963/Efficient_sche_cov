from robot_class import location_extraction
from robot_class import routing_strategy_extraction
from robot_class import single_node_move

import numpy.linalg as LA
import sys

sys.path.append(r'../communication_model')
from commu_model import optimal_routing
from commu_model import transmission
from routing_present import routing_graph

N = 11
K = 1

def active_team_move(robot_list, moving_robot_index, target):

    'this function updates locations of robots in active leader team'
    'moving robot index is an orded set, the first one is nearest to the AP,'
    'the last one is leader'

    sigma = 0.1
    # moving radius, the less, the accurate
    location = location_extraction(robot_list)
    new_location = location.copy()
    # be sure to use .copy
    team_length = len(moving_robot_index)
    routing_strategy = routing_strategy_extraction(robot_list)
    print('moving robot index:')
    print(moving_robot_index)
    while(1):
        for i in range(team_length):
            if(i<team_length-1):
                new_location[moving_robot_index[i]] = \
                    single_node_move(location, moving_robot_index[i],
                                     location[moving_robot_index[i+1]], routing_strategy, sigma)
            else:
                new_location[moving_robot_index[i]] = single_node_move(location, moving_robot_index[i],
                                     target, routing_strategy, sigma)

        if(max(LA.norm((new_location-location), axis=1))<=0.01):
            # if new_location is the same as current location, break
            print(LA.norm((new_location-location), axis=1))
            print('BREAK1')
            break
        else:
            print('calculate optimal routing')
            res, routing_strategy = optimal_routing(new_location)
            # calculate routing_strategy for current locations
            # if solvable(find optimal solution), record these locations to robots
            # and update routing strategies.
            if(res['status']=='optimal'):
                print('this is optimal')
                print(new_location[moving_robot_index[-1]])
                for i, x in enumerate(robot_list):
                    if (i < N):
                        x.location_update(location[i])
                        x.routing_strategy_update(routing_strategy[i])
            else:
            # otherwise, break, keep locations and routing strategies of last time
                print('BERAK2')
                break
            location = new_location.copy()
        routing_graph(location, transmission(location, routing_strategy), N, K)
    return robot_list
