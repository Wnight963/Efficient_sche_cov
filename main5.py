# the final version of a sequence of task
import sys
sys.path.append(r'motion_model')
sys.path.append(r'communication_model')

import numpy as np
import pickle

from move_model import scheduling_for_single_task

f = open('robots_initial.pckl', 'rb')
robots = pickle.load(f)
f.close()

for x in robots:
    x.pre_role = "redundant_node"
    x.location /= 2
    print(x.number, x.role, x.pre_role, x.location)
robots[-1].pre_role = 'AP'
# robots
# task = [[3,3], [4,4]]
# task = np.array(task)
# task_completion_progress = [0 for x in task]
# task
# task = []
# for i in range(5):
#     center = []
#     for j in range(10):
#         center.append([3.5+1*i, 3.25+0.5*j])
#     if i%2==0:
#         task.extend(center)
#     else:
#         center.reverse()
#         task.extend(center)

task = []
for i in range(5):
    center = []
    for j in range(10):
        center.append([3.5+1*i, 3.25+0.5*j])
    if i%2==0:
        task.extend(center)
    else:
        center.reverse()
        task.extend(center)
# task = [[3.5 ,3.25], [4.5 ,-0.5]]
task = np.array(task)
from move_model import leader_coverage
robots = leader_coverage(robots, task)
#
# for i in range(len(task)):
#     robots = scheduling_for_single_task(robots, task[i])
#
print('*******************************************************')
for x in robots:
    print(x.number, x.role, x.location)
    # name = 'robots_after_' + str(i+1) + 'st_task.pckl'
    # f = open(name, 'wb')
    # pickle.dump(robots, f)
    # f.close()