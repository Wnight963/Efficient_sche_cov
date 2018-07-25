import numpy as np
import copy

a = np.array([[1,2], [3,4]])
b = np.random.uniform(0,1,size=[2,2])
b = a.copy()

print(id(a))
print(id(b))

a = b.copy()
print(id(a))

print(1!=0)