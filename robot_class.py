import numpy as np
import numpy.linalg as LA
import sys

sys.path.append(r'communication_model')

N = 10
K = 1

class ROBOT:
    'data structure of robot'

    role_set = set(['node', 'redundant_node', 'junction', 'leaf', 'leader'])
    def __init__(self, number, location, routing_strategy, role='redundant_node'):
        self.number = number
        self.location = location
        self.routing_strategy = routing_strategy
        self.role = role

    def location_update(self, new_location):
        self.location = new_location
    def role_update(self, new_role):
        if(new_role in self.role_set):
            self.role = new_role
        else:
            print("Error!")




def location_extraction(robot_list):
    M = len(robot_list)
    location = robot_list[0].location
    for i in range(1, M):
        location = np.vstack((location, robot_list[i].location))
    return location


def leader_election(robot_list, target): # waiting to be modefied into distributed fashion

    #input: a list of robot; output: leader of these robots via consensus

    location = location_extraction(robot_list)
    distance = [LA.norm(x-target) for x in location]
    leader_index = distance.index(min(distance[0:N]))
    robot_list[leader_index].role_update('leader')
    return robot_list, leader_index


def leader_move(robot_list,leader_index, target):

    #leader moves towards the target, until communication constraints are breaked
    #to determine the farest position leader can reach, we use a binary search

    from commu_model import optimal_routing
    location = location_extraction(robot_list)
    a, b = robot_list[leader_index].location, target
    while(1):
        new_location = (a+b)/2
        if(LA.norm(new_location-a)<=0.02):
            break
        else:
            location[leader_index] = new_location
            res, unuse = optimal_routing(location, leader_index)
            if(res['status']=='optimal'):
                a = new_location
            else:
                b = new_location
    robot_list[leader_index].location_update(new_location)

    return robot_list
