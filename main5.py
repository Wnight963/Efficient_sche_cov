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
    print(x.number, x.role, x.pre_role, x.location)
robots[-1].pre_role = 'AP'
# robots
task = [[3, 2.0], [4, 2.5], [4, 3.5], [3, 4], [5, 2], [5, 4]]
task = [np.array(x) for x in task]
task_completion_progress = [0 for x in task]
# task


print(task_completion_progress)
for i in range(len(task)):
    if task_completion_progress[i]==0:
        robots = scheduling_for_single_task(robots, task[i])
    print('*******************************************************')
    # for x in robots:
    #     print(x.number, x.role, x.location)
    # name = 'robots_after_' + str(i+1) + 'st_task.pckl'
    # f = open(name, 'wb')
    # pickle.dump(robots, f)
    # f.close()

