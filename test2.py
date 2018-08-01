import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Circle

# x = np.random.random(size = 200)
# y = np.random.random(size = 200)




fig = plt.figure()
ax = fig.add_subplot(111)


cir1 = Circle(xy = (0.0, 0.0), radius=2, alpha=0.5)
ax.add_patch(cir1)

x, y = 0, 0
ax.plot(x, y, 'ro')

plt.axis('scaled')
# ax.set_xlim(-4, 4)
# ax.set_ylim(-4, 4)
plt.axis('equal')   #changes limits of x or y axis so that equal increments of x and y have the same length

plt.show()