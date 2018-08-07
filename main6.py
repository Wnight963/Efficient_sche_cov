import sys
sys.path.append(r'motion_model')
sys.path.append(r'communication_model')
import numpy as np
import pickle
from move_model import scheduling_for_single_task

# f = open('robots_after_coverage_.pckl', 'rb')
# robots= pickle.load(f)
# f.close()

f = open('robots_initial.pckl', 'rb')
robots = pickle.load(f)
f.close()

for x in robots:
    x.pre_role = "redundant_node"
    x.location /= 3
    print(x.number, x.role, x.pre_role, x.location)
robots[-1].pre_role = 'AP'


worker = [x.number for x in robots if x.role!='redundant_node']
from robot_class import robot_network_extraction
G = robot_network_extraction(robots)
H = G.subgraph(worker)
for nod in H.in_degree:
    index = nod[0]
    deg = nod[1]
    if deg==1:
        robots[index].role_update('leaf')
    elif deg>=3:
        robots[index].role_update('junction')


positions = [[4.2, 4], [4.5, 6], [3.7, 7], [7, 5.5], [7.2, 7]]
positions = np.array(positions)
for pos in positions:
    robots = scheduling_for_single_task(robots, pos)






