import numpy as np
import numpy.linalg as LA
a = np.array([[1,4],[3,9],[2,5],[10,12]])
c = [0,2]
b = a[c]

print(LA.norm(a))

