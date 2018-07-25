import numpy as np
import numpy.linalg as LA
import sys

sys.path.append(r'communication_model')

N = 11
K = 1

class ROBOT:
    'data structure of robot'

    role_set = set(['node', 'redundant_node', 'junction', 'leaf', 'leader'])
    def __init__(self, number, location, routing_strategy=np.zeros([1, N+K]), role='redundant_node'):
        self.number = number
        self.location = location
        self.routing_strategy = routing_strategy
        self.role = role
    # routing_strategy: np.array(), shape 1*(N+K)

    def location_update(self, new_location):
        self.location = new_location
    def role_update(self, new_role):
        if(new_role in self.role_set):
            self.role = new_role
        else:
            print("Error!")
    def routing_strategy_update(self, new_routing_strategy):
        self.routing_strategy = new_routing_strategy
    # def routing_strategy_update(self, new_routing_strategy):





def location_extraction(robot_list):
    M = len(robot_list)
    location = robot_list[0].location
    for i in range(1, M):
        location = np.vstack((location, robot_list[i].location))
    return location

def routing_strategy_extraction(robot_list):
    M = len(robot_list)
    routing_strategy = robot_list[0].routing_strategy
    for i in range(1, M):
        routing_strategy = np.vstack((routing_strategy, robot_list[i].routing_strategy))
    return routing_strategy

def leader_index_extraction(robot_list):
    for x in robot_list:
        if(x.role=='leader'):
            return x.number

def leader_election(robot_list, target): # waiting to be modefied into distributed fashion

    #input: a list of robot; output: leader of these robots via consensus

    location = location_extraction(robot_list)
    distance = [LA.norm(x-target) for x in location]
    leader_index = distance.index(min(distance[0:N]))
    robot_list[leader_index].role_update('leader')
    return robot_list, leader_index

def recruit_election(robot_list):
    routing_strategy = routing_strategy_extraction(robot_list)
    leader_index = leader_index_extraction(robot_list)
    T = list(routing_strategy[leader_index,:])
    return T.index(min(T))


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
            res, routing_result = optimal_routing(location, leader_index)
            if(res['status']=='optimal'):
                robot_list[leader_index].location_update(new_location)
                for i in range(N):
                    robot_list[i].routing_strategy_update(routing_result[i, :])
                a = new_location
            else:
                b = new_location
    return robot_list
