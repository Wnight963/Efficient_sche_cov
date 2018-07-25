import numpy as np
import numpy.linalg as LA
import sys

sys.path.append(r'communication_model')

from commu_model import ci

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
    # location: np.ndarray, raw represents robot, two colomns.

    def location_update(self, new_location):
        self.location = new_location
    def role_update(self, new_role):
        if(new_role in self.role_set):
            self.role = new_role
        else:
            print("Error!")
    def routing_strategy_update(self, new_routing_strategy):
        self.routing_strategy = new_routing_strategy






def location_extraction(robot_list):
    'input: a list of robots, output: location of robots, np.ndarray, shape (robot number, 2)'
    M = len(robot_list)
    location = robot_list[0].location
    for i in range(1, M):
        location = np.vstack((location, robot_list[i].location))
    return location

def routing_strategy_extraction(robot_list):
    'output: routing strategy of robots from 0 to N-1, the last K are APs and are overlooked'
    M = len(robot_list)
    routing_strategy = robot_list[0].routing_strategy
    for i in range(1, M):
        routing_strategy = np.vstack((routing_strategy, robot_list[i].routing_strategy))
    return routing_strategy[0:N]

def leader_index_extraction(robot_list):
    'return number of robot whose role is "leader"'
    for x in robot_list:
        if(x.role=='leader'):
            return x.number

def leader_election(robot_list, target): # waiting to be modefied into distributed fashion

    'input: a list of robot; output: leader of these robots via consensus'

    location = location_extraction(robot_list)
    distance = [LA.norm(x-target) for x in location]
    leader_index = distance.index(min(distance[0:N]))
    robot_list[leader_index].role_update('leader')
    return robot_list, leader_index

def recruit_election(robot_list):

    'input: a list of robot; output: recruit of these robots via consensus'

    routing_strategy = routing_strategy_extraction(robot_list)
    leader_index = leader_index_extraction(robot_list)
    T = list(routing_strategy[leader_index,:])[0:N+K-1]
    recruit_index = T.index(min(T))
    robot_list[recruit_index].role_update('node')
    return robot_list, recruit_index


def leader_move(robot_list,leader_index, target):

    'leader moves towards the target, until communication constraints are breake'
    'to determine the farest position leader can reach, we use a binary search'

    from commu_model import optimal_routing
    location = location_extraction(robot_list)
    a, b = robot_list[leader_index].location, target
    while(1):
        new_location = (a+b)/2
        if(LA.norm(new_location-a)<=0.02):
            break
        else:
            location[leader_index] = new_location
            res, routing_result = optimal_routing(location)
            if(res['status']=='optimal'):
                robot_list[leader_index].location_update(new_location)
                for i in range(N):
                    robot_list[i].routing_strategy_update(routing_result[i, :])
                a = new_location
            else:
                b = new_location
    return robot_list


def single_node_move(location_of_robot, single_moving_node_index, destination, routing_strategy, sigma):

    'robot moves towards the destination, with given communication constraints obeyed'
    'to determine the farest position leader can reach, we use a binary search'
    'return new location of that moving node'



    location = location_of_robot.copy()
    # this is important! to copy a list in python, The operator [:] returns a slice of a sequence.
    # Slicing a portion of a list: create a new list. In this way, changing location will not changes
    # location_of_robot as the same time.

    # print('before moving:')
    # print(location[single_moving_node_index])
    # print('target:')
    # print(destination)

    a, b = location[single_moving_node_index], destination
    b = (sigma/(LA.norm(b-a)))*(b-a) + a
    while(1):
        new_location = (a+b)/2
        if(LA.norm(new_location-a)<=0.0002):
            # print('after moving:')
            # print(a)
            return a
        else:
            location[single_moving_node_index] = new_location
            res = ci(location, single_moving_node_index, routing_strategy)
            # check if the packet routing condition is obeyed
            if(res>=0):
                a = new_location
            else:
                b = new_location

