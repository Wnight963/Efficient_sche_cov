import numpy as np
import matplotlib.pyplot as plt



for i in range(10):
    plt.cla()
    plt.axis([0, 10, 0, 1])
    y = np.random.random()
    plt.scatter(i, y)
    plt.pause(0.5)

plt.show()



