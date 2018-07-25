import numpy as np
import copy

a = np.array([[1,2], [3,4]])
b = np.random.uniform(0,1,size=[2,2])
b = a.copy()
b[0,0] = 44444444
print(id(a))
print(id(b))
print(a)
print(b)
