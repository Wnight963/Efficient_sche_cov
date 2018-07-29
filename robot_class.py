import numpy as np
import networkx as nx
import numpy.linalg as LA
import sys

sys.path.append(r'communication_model')

from commu_model import ci

N = 11
K = 1


class ROBOT:
    'data structure of robot'

    role_set = set(['node', 'redundant_node', 'junction', 'leaf', 'leader', 'AP'])
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


def channel_matrix_extraction(robot_list):

    'output: channel matrix between robots, a np.array, shape: N*(N+K)'

    from channel_capa import Capacity
    x = location_extraction(robot_list)
    R = np.zeros([N, N + K])
    for i in range(N):
        for j in range(N + K):
            R[i, j] = Capacity(x[i], x[j])
    return R

def routing_strategy_extraction(robot_list):

    'output: routing strategy of robots from 0 to N-1, the last K are APs and are overlooked'
    'output is a np.array, shape: N*(N+K)'

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


def redundant_node_index_extraction(robot_list):

    'return list of redundant node index'

    res = []
    for x in robot_list:
        if(x.role=='redundant_node'):
            res.append(x.number)
    return res


def leader_election(robot_list, target): # waiting to be modefied into distributed fashion

    'input: a list of robot; output: leader of these robots via consensus'

    distance = [LA.norm(x.location-target) for x in robot_list if x.role=='redundant_node' or 99999]
    leader_index = distance.index(min(distance))
    robot_list[leader_index].role_update('leader')
    return robot_list, leader_index


def recruit_election(robot_list):

    'input: a list of robot; output: recruit of these robots via consensus'

    import random
    redundant_node_index = redundant_node_index_extraction(robot_list)
    if(len(redundant_node_index)):
        recruit_index = random.sample(redundant_node_index, 1)[0]
        robot_list[recruit_index].role_update('node')
        return robot_list, recruit_index
    else:
        print("there is no redundant node, all nodes are occupied!")
        return False, False


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

    print("leader moves successfully!")
    print(robot_list[leader_index].location)
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
    sigma = min(sigma, LA.norm(b-a))
    b = (sigma/(LA.norm(b-a)))*(b-a) + a
    while(1):
        new_location = (a+b)/2
        if(LA.norm(new_location-a)<=0.02):
            # print('after moving:')
            # print(a)
            return a
        else:
            location[single_moving_node_index] = new_location
            res = [ci(location, i, routing_strategy) for i in range(N)]
            # check if the packet routing condition is obeyed, for all robots after a single
            # one moves
            if(min(res)>=-0.01):
                # using a small negetive number rather than zero is because
                # a slightly breaking of communication constraint is not very harmful
                # and allows robots to move more efficiently
                a = new_location
            else:
                b = new_location


def robot_network_extraction(robots):

    'construct a robot communication network, nodes are robots, and edges exist if T*R>0'
    'return a nx.DiGraph'

    import networkx as nx
    G = nx.DiGraph()
    G.add_nodes_from(range(N + K))
    T = routing_strategy_extraction(robots)
    R = channel_matrix_extraction(robots)
    transmission_data = T * R
    edges = [(j, i) for i in range(N) for j in range(N + K) if transmission_data[i, j] > 0]
    G.add_edges_from(edges)
    return G


def subgroup_extraction(original_list, word):

    "input: original list, like:['R', 'N', 'N', 'J', 'N', 'N', 'J', 'N', 'N']"
    "word, a list of what word to extract, like: ['J', 'J']"
    "return [['R', 'N', 'N'], ['J', 'N', 'N'], ['J', 'N', 'N']]"

    b = original_list
    print(b)
    c = [i for i, x in enumerate(b) if x in word]
    sum = [b[0:c[0]]]
    for i in range(len(c)):
        if (i == len(c) - 1):
            sum.append(b[c[i]:])
        else:
            sum.append(b[c[i]:c[i + 1]])
    return sum


def secondary_leader_team_construction(robots, recruit_index, recruit_inducer_index,
                                       after_leader_election=False):

    'construct the secondary leader team, source node: the recruit,'
    'target node: robot who induces recruit'

    G = robot_network_extraction(robots)
    shortest_path = nx.shortest_path(G, source=N+K-1, target=recruit_inducer_index)
    # find the shortest path from AP to robot who induces recruit
    shortest_path[0] = recruit_index
    # then replace the AP with previously recruit robot. In this way, the fault that there may
    # exist no path from recruit to recruit_inducer is avoided.
    roles_in_2nd_leader_team = [robots[x].role for x in shortest_path]
    if(after_leader_election==True):
        break_role = ['junction', 'leader']
        subgroup_of_2nd_leader_team = subgroup_extraction(roles_in_2nd_leader_team, break_role)
    else:
        break_role = ['junction', 'leaf']
        subgroup_of_2nd_leader_team = subgroup_extraction(roles_in_2nd_leader_team, break_role)
    return subgroup_of_2nd_leader_team


