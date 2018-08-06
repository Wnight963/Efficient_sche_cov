import numpy as np
import matplotlib.pyplot as plt
import pickle
import datetime
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle

fig = plt.figure("routing")
ax = fig.add_subplot(111)

f = open('user_equip_location.pckl', 'rb')
user = pickle.load(f)
f.close()

global deteced_user
deteced_user = []
# global variable, all the detected users


def if_is_detected(x, user):

    'determine if the user equipment is detected by UAVS'
    'x: locations of UAV, user: location of user'

    distance = x - user
    photo_range = np.array([0.5, 0.25])
    dectected = [list(abs(n) < photo_range) for n in distance]
    res = [True, True] in dectected
    return res



def routing_graph(x, transmission, N, K):

    'input: x:locations of robots'

    ax.cla()
    # ax.grid(True, linestyle="-.", color="r", linewidth="3")
    ax.axis([3, 8, 3, 8])
    major_ticks = np.arange(3, 8, 1)
    minor_ticks = np.arange(3, 8, 0.5)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks)
    ax.grid(which='both')
    ax.scatter(x[:, 0], x[:, 1])

    global deteced_user
    residual_user = [x for x in user if x not in np.array(deteced_user)]
    for n in residual_user:
        if if_is_detected(x, n):
            deteced_user.append(n)

    if len(deteced_user):
        ax.scatter(np.array(deteced_user)[:,0], np.array(deteced_user)[:,1], linewidths=1)


    for i in range(len(x)):
        # cir = Circle(xy=(x[i, 0], x[i, 1]), radius=0.5, alpha=0.5)
        # ax.add_patch(cir)
        xy = (x[i, 0] - 0.5, x[i, 1] - 0.25)
        rec = Rectangle(xy, height=0.5, width=1, fill=False)
        ax.add_patch(rec)
        # draw a circle around UAVs
    # task = [[3,3], [4,4]]
    # task = np.array(task)

    task = []
    for i in range(5):
        center = []
        for j in range(10):
            center.append([3.5 + 1 * i, 3.25 + 0.5 * j])
        if i % 2 == 0:
            task.extend(center)
        else:
            center.reverse()
            task.extend(center)
    task = np.array(task)

    ax.scatter(task[:, 0], task[:, 1], marker='x')
    n = range(N + K)
    for i, txt in enumerate(n):
        ax.annotate(txt, (x[i, 0], x[i, 1]))  # number the points

    for i in range(N):
        for j in range(N + K):
            if transmission[i, j] > 0:
                a = x[[i, j], 0]
                b = x[[i, j], 1]
                ax.plot(a, b, linewidth=50 * transmission[i, j])
                # thickness denotes transmmision rate

    plt.ion()
    plt.pause(0.01)
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