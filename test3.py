# import numpy as np
# import pickle
#
# x = np.array([[1,2], [3,4], [5,6]])
# user = np.array([1, 1])
#
# print(abs(user)<x[2])
#
# def if_is_detected(x, user):
#     distance = x - user
#     photo_range = np.array([1,2])
#     dectected = [list(abs(n)<photo_range) for n in distance]
#     res = [True, True] in dectected
#     return res
#
# print(if_is_detected(x, user))
#
# print(user not in x)
#


import numpy as np
import pickle
import matplotlib.pyplot as plt

task = []
for i in range(5):
    center = []
    for j in range(10):
        center.append([3.2+0.4*i, 3.1+0.2*j])
    if i%2==0:
        task.extend(center)
    else:
        center.reverse()
        task.extend(center)
task = np.array(task)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.axis([3,5,3,5])
major_ticks = np.arange(3, 5, 0.4)
minor_ticks = np.arange(3, 5, 0.2)
ax.set_xticks(major_ticks)
ax.set_yticks(major_ticks)
ax.set_yticks(minor_ticks)
ax.grid(which='both')
plt.scatter(task[:,0], task[:,1])
plt.show()






