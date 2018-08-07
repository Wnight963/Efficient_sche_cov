import sys
sys.path.append(r'../communication_model')
sys.path.append(r'..')
from robot_class import location_extraction
from robot_class import routing_strategy_extraction
from robot_class import robot_network_extraction
from robot_class import single_node_move
from robot_class import leader_move
from robot_class import leader_election
from robot_class import recruit_election
from robot_class import secondary_leader_team_construction
from robot_class import subgroup_index_extraction
from commu_model import optimal_routing
from commu_model import transmission
from routing_present import routing_graph_1
from routing_present import routing_graph_2


import numpy as np
import networkx as nx
import numpy.linalg as LA

N = 11
K = 1

# task = [[3,3], [4,4]]
# task = [np.array(x) for x in task]

task = []
for i in range(5):
    center = []
    for j in range(10):
        center.append([3.5 + 1 * i, 3.25 + 0.5 * j])
    if i%2==0:
        task.extend(center)
    else:
        center.reverse()
        task.extend(center)
task = np.array(task)

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
    # print('moving robot index:')
    # print(moving_robot_index)
    while(1):
        for i in range(team_length):
            if(i<team_length-1):
                # print('before moving: %d' % moving_robot_index[i])
                # print(location[moving_robot_index[i]])

                new_location[moving_robot_index[i]] = \
                    single_node_move(location, moving_robot_index[i],
                                     location[moving_robot_index[i+1]], routing_strategy, sigma)
                # print('after moving: %d' % moving_robot_index[i])
                # print(new_location[moving_robot_index[i]])
            else:
                # print('before moving: %d' % moving_robot_index[i])
                # print(location[moving_robot_index[i]])
                new_location[moving_robot_index[i]] = single_node_move(location, moving_robot_index[i],
                                     target, routing_strategy, sigma)
                # print('after moving: %d' % moving_robot_index[i])
                # print(new_location[moving_robot_index[i]])
        if(max(LA.norm((new_location-location), axis=1))<=0.002):
            # if new_location is the same as current location, break
            print('New location is very near to current one, moving is not necessary')
            break
        else:
            # print('calculate optimal routing')
            res, routing_strategy = optimal_routing(new_location)
            # calculate routing_strategy for current locations
            # if solvable(find optimal solution), record these locations to robots
            # and update routing strategies.

            if(res['status']=='optimal'):
                # print('this is optimal')
                for i, x in enumerate(robot_list):
                    if (i < N):
                        x.location_update(location[i])
                        x.routing_strategy_update(routing_strategy[i])
            else:
            # otherwise, break, keep locations and routing strategies of last time
            #     print('fail to find optimal routing!')
            #     print('BERAK2')
                break
            location = new_location.copy()
        routing_graph_1(location, transmission(location, routing_strategy), N, K)
        routing_graph_2(location, transmission(location, routing_strategy), N, K)
    # print("ACTIVE MOVING TERMINATED!")
    return robot_list


def scheduling_for_single_task(robot_list, target):
    'this function simulates the movement of an existing robot network when given a'
    'new task'

    robots = robot_list
    robots, leader_index = leader_election(robots, target)
    print('leader is %d' % leader_index)

    if (robots[leader_index].pre_role == 'leaf'):
        robots, recruit_index = recruit_election(robots)
        secondary_leader_team_index = secondary_leader_team_construction(
            robots, recruit_index, leader_index)
        print('secondary leader team:')
        print(secondary_leader_team_index)
        for x in secondary_leader_team_index:
            print([robots[h].role for h in x])
        ########### once the leader is elected and it's previous role is a leaf node
        ########### a recruit election is immediately induced to release the leader from
        ########### its previous location
        # secondary_leader_team_index.append([leader_index])
        for i in range(len(secondary_leader_team_index)):
            if i<len(secondary_leader_team_index)-1:

                # current_location = robots[secondary_leader_team_index[i + 1][0]].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                       (distance_to_current_location))]

                destination = robots[secondary_leader_team_index[i + 1][0]].location
            else:

                # current_location = robots[leader_index].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                       (distance_to_current_location))]

                destination = robots[leader_index].location
            robots = active_team_move(robots, secondary_leader_team_index[i], destination)
            #######
            ####### if the subgroup's destination is a leaf node, we just change the destination
            ####### to the nearest task
            if (i < len(secondary_leader_team_index) - 1):
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
            else:
                tmp_role = robots[leader_index].pre_role
            robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
        for i in range(len(secondary_leader_team_index)):
            if i>0:
                robots[secondary_leader_team_index[i][0]].role_update('node')
        ###########
        primary_leader_team = [leader_index]
        ########## primary team now has only one member leader
        robots = active_team_move(robots, primary_leader_team, target)
        # for x in secondary_leader_team_index:
        #     for i in x:
        #         print(i, robots[i].role)
        # for i in primary_leader_team:
        #     print(i,robots[i].role)

        ########## primary team moves and gets trapped in local stationary point
        while (LA.norm(robots[leader_index].location - target) >= 0.2):
            robots, recruit_index = recruit_election(robots)
            secondary_leader_team_index = secondary_leader_team_construction(
                robots, recruit_index, primary_leader_team[0])
            print('**********secondary leader team:***********')
            print(secondary_leader_team_index)
            for x in secondary_leader_team_index:
                print([robots[h].role for h in x])
            for i in range(len(secondary_leader_team_index) - 1):
                ####### -1 because the last one will join primary leader team then moves
                # current_location = robots[secondary_leader_team_index[i + 1][0]].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                           (distance_to_current_location))]
                destination = robots[secondary_leader_team_index[i + 1][0]].location
                robots = active_team_move(robots, secondary_leader_team_index[i], destination)

                # if (i < len(secondary_leader_team_index) - 2):
                #     tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
                # else:
                #     tmp_role = robots[leader_index].pre_role
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
                robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
            for i in range(len(secondary_leader_team_index) - 1):
                if i > 0:
                    robots[secondary_leader_team_index[i][0]].role_update('node')
            primary_leader_team.insert(0, secondary_leader_team_index[-1][0])
            robots[primary_leader_team[0]].role_update("node")
            print("***********primary leader team************")
            print(primary_leader_team)
            robots = active_team_move(robots, primary_leader_team, target)
        if (LA.norm(robots[leader_index].location - target) < 0.2):
            print("task completed!")
        for x in robots:
            if (x.role == 'leader'):
                x.role_update('leaf')


    elif (robots[leader_index].pre_role == 'node' or robots[leader_index].pre_role == 'junction'):

        robots, recruit_index = recruit_election(robots)
        secondary_leader_team_index = secondary_leader_team_construction(
            robots, recruit_index, leader_index)
        print('secondary leader team:')
        print(secondary_leader_team_index)
        for x in secondary_leader_team_index:
            print([robots[h].role for h in x])
        ########### once the leader is elected and it's previous role is a junction or node
        ########### a recruit election is immediately induced to release the leader from
        ########### its previous location
        # secondary_leader_team_index.append([leader_index])
        for i in range(len(secondary_leader_team_index)):
            if i < len(secondary_leader_team_index) - 1:


                # current_location = robots[secondary_leader_team_index[i + 1][0]].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                       (distance_to_current_location))]


                destination = robots[secondary_leader_team_index[i + 1][0]].location
            else:


                # current_location = robots[leader_index].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                       (distance_to_current_location))]


                destination = robots[leader_index].location
            robots = active_team_move(robots, secondary_leader_team_index[i], destination)

            #######
            ####### if the subgroup's destination is a leaf node, we just change the destination
            ####### to the nearest task
            if (i < len(secondary_leader_team_index) - 1):
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
            else:
                tmp_role = 'junction'
            robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
        for i in range(len(secondary_leader_team_index)):
            if i > 0:
                robots[secondary_leader_team_index[i][0]].role_update('node')
        ###########
        primary_leader_team = [leader_index]
        ########## primary team now has only one member leader
        robots = active_team_move(robots, primary_leader_team, target)
        # for x in secondary_leader_team_index:
        #     for i in x:
        #         print(i, robots[i].role)
        # for i in primary_leader_team:
        #     print(i,robots[i].role)

        ########## primary team moves and gets trapped in local stationary point
        while (LA.norm(robots[leader_index].location - target) >= 0.2):
            robots, recruit_index = recruit_election(robots)
            secondary_leader_team_index = secondary_leader_team_construction(
                robots, recruit_index, primary_leader_team[0])
            print('**********secondary leader team:***********')
            print(secondary_leader_team_index)
            for x in secondary_leader_team_index:
                print([robots[h].role for h in x])
            for i in range(len(secondary_leader_team_index) - 1):
                # -1 because the last one will join primary leader team then moves
                # current_location = robots[secondary_leader_team_index[i + 1][0]].location
                # distance_to_current_location = [LA.norm(x - current_location) for x in task]
                # destination = task[distance_to_current_location.index(min
                #                                                       (distance_to_current_location))]
                destination = robots[secondary_leader_team_index[i + 1][0]].location
                robots = active_team_move(robots, secondary_leader_team_index[i], destination)
                # if (i < len(secondary_leader_team_index) - 2):
                #     tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
                # else:
                #     tmp_role = robots[leader_index].pre_role
                tmp_role = robots[secondary_leader_team_index[i + 1][0]].role
                robots[secondary_leader_team_index[i][-1]].role_update(tmp_role)
            for i in range(len(secondary_leader_team_index) - 1):
                if i > 0:
                    robots[secondary_leader_team_index[i][0]].role_update('node')
            primary_leader_team.insert(0, secondary_leader_team_index[-1][0])
            robots[primary_leader_team[0]].role_update("node")
            print("***********primary leader team************")
            print(primary_leader_team)
            robots = active_team_move(robots, primary_leader_team, target)
        if (LA.norm(robots[leader_index].location - target) < 0.2):
            print("task completed!")
        for x in robots:
            if (x.role == 'leader'):
                x.role_update('leaf')

    else:
        # don't bother to use primary leader team and secondary leader team
        robots = leader_move(robots, leader_index, target)
        moving_robot_index = [leader_index]
        while (LA.norm(robots[leader_index].location - target) >= 0.2):
            robots, recruit_index = recruit_election(robots)
            print("recruit_index:")
            print(recruit_index)
            moving_robot_index.insert(0, recruit_index)
            robots = active_team_move(robots, moving_robot_index, target)
        for x in robots:
            if (x.role == 'leader'):
                x.role_update('leaf')
    return robots





def leader_coverage(robot_list, target):

    """

    :param robot_list:
    :param target: a sequence of locations to visit so that a zone can be covered
    :return: robot_list when coverage is done
    """

    import matplotlib.pyplot as plt

    robots = robot_list
    robots, leader_index = leader_election(robots, target[0])
    worker = [leader_index, N+K-1]
    for point in target:
        robots = leader_move(robots, leader_index, point)

        while (LA.norm(robots[leader_index].location - point) >= 0.05):
            robots, recruit_index = recruit_election(robots)
            print('recruit:')
            print(recruit_index)
            G = robot_network_extraction(robots)
            H = G.subgraph(worker)
            H = nx.relabel_nodes(H, {N+K-1: recruit_index})

            # nx.draw(H, with_labels=True)
            # plt.show()

            # replace AP by the recruit
            for nod in H.out_degree:
                index = nod[0]
                out_deg = nod[1]
                if robots[index].role=='node' and out_deg>2:
                    robots[index].role_update('junction')
                    print("%d is a junction node!!!! out degree is %d" % (index, out_deg))
                    print('successors are:')
                    print([y for y in H.successors(index)])
            # identify all junction node




            shortest_path = nx.shortest_path(H, source=recruit_index, target=leader_index)
            roles_in_2nd_leader_team = [robots[x].role for x in shortest_path]
            break_role = ['junction']
            subgroup_of_2nd_leader_team_index = subgroup_index_extraction(roles_in_2nd_leader_team, break_role)
            moving_robot_index = [shortest_path[x[0]:(x[-1] + 1)] for x in subgroup_of_2nd_leader_team_index]
            # shortest path from AP to leader
            print('moving robot index:')
            print(moving_robot_index)


            for i in range(len(moving_robot_index)):
                if i<len(moving_robot_index)-1:
                    target = robots[moving_robot_index[i+1][0]].location
                else:
                    target = point
                robots = active_team_move(robots, moving_robot_index[i], target)

            # worker = [x for x in worker if LA.norm(robots[x].location) > 0.4]
            for nod in H.out_degree:
                index = nod[0]
                if robots[index].role!='leader':
                    robots[index].role_update('node')
            worker.insert(0, recruit_index)
            # if max(LA.norm((location_extraction(robots) - old_location), axis=1))<0.05:



    return robots
