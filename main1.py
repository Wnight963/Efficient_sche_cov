# the final version of a sequence of task
import numpy as np
import pickle
import sys
sys.path.append(r'motion_model')
sys.path.append(r'communication_model')
from move_model import scheduling_for_single_task

f = open('robots_initial.pckl', 'rb')
robots = pickle.load(f)
f.close()
for x in robots:
    x.pre_role = "redundant_node"
    print(x.number, x.role, x.pre_role, x.location)
robots[-1].pre_role = 'AP'

task = []
for i in range(5):
    center = []
    for j in range(2):
        center.append([3.5+1*i, 3.25+0.5*9*j])
    if i%2==0:
        task.extend(center)
    else:
        center.reverse()
        task.extend(center)
task = np.array(task)

for i in range(len(task)):
