# import numpy as np
# import pickle
# import matplotlib.pyplot as plt
# user = 5*np.random.random([400,2]) + np.array([3,3])
# plt.scatter(user[:,0], user[:,1])
# plt.show()
# f = open('user_equip_location.pckl', 'wb')
# pickle.dump(user, f)
# f.close()


# a=10
# b=0
# try:
#     c=a/b
#     print(c)
# except ZeroDivisionError as err:
#     print(err)
# print("done")

import sys
sys.path.append(r'motion_model')
sys.path.append(r'communication_model')

import numpy as np
import pickle
import networkx as nx
import matplotlib.pyplot as plt
from move_model import scheduling_for_single_task

f = open('robots_after_coverage_.pckl', 'rb')
robots= pickle.load(f)
f.close()


worker = [x.number for x in robots if x.role!='redundant_node']
from robot_class import robot_network_extraction
G = robot_network_extraction(robots)
H = G.subgraph(worker)
for nod in H.in_degree:
    index = nod[0]
    deg = nod[1]
    print("%d node has in degree %d" % (index, deg))
    if deg==1:
        robots[index].role_update('leaf')
    elif deg>=3:
        robots[index].role_update('junction')


task = np.array([4.5, 7])
robots = scheduling_for_single_task(robots, task)






