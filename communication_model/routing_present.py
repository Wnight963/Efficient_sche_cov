import numpy as np
import matplotlib.pyplot as plt
import datetime

plt.figure("routing")
def routing_graph(x, transmission, N, K):

    'input: x:locations of robots'


    plt.cla()
    plt.scatter(x[:,0],x[:,1])
    task = [[3, 2.0], [4, 2.5], [4, 3.5], [3, 4], [5, 2], [5, 4]]
    task = np.array(task)
    plt.scatter(task[:,0], task[:,1])
    n = range(N+K)
    for i,txt in enumerate(n):
        plt.annotate(txt, (x[i,0],x[i,1]))             # number the points

    for i in range(N):
        for j in range(N+K):
            if transmission[i,j]>0:
                a = x[[i,j],0]
                b = x[[i,j],1]
                plt.plot(a, b, linewidth=25*transmission[i,j])       # thickness denotes transmmision rate
    plt.ion()
    plt.pause(0.02)
    # plt.close()
    # plt.show()

    # now_time = datetime.datetime.now()
    # name = datetime.datetime.strftime(now_time,'%Y-%m-%d %H_%M_%S')
    # plt.savefig('picture/'+name)
    return

def routing_graph_for_robot_list(robot_list):

    'similar to the above function, while the input is a robot list'

    from routing_present import routing_graph
    from robot_class import routing_strategy_extraction
    from commu_model import transmission
    from robot_class import location_extraction
    import sys
    sys.path.append(r'motion_model')
    sys.path.append(r'communication_model')

    N = 11
    K = 1
    routing = routing_strategy_extraction(robot_list)
    location = location_extraction(robot_list)
    transmission = transmission(location, routing)
    routing_graph(location, transmission, N, K)
    return