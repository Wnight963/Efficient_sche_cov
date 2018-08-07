import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure('1')
ax = fig.add_subplot(111)
ax.axis([3,8,3,8])
positions = [[4.2, 4], [4.5, 6], [3.7, 7], [7, 5.5], [7.2, 7]]
positions = np.array(positions)
ax.scatter(positions[:,0], positions[:,1])
plt.show()