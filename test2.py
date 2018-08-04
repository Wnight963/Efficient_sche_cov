import numpy as np
import pickle
import matplotlib.pyplot as plt
user = 2*np.random.random([400,2]) + np.array([3,3])
plt.scatter(user[:,0], user[:,1])
plt.show()
f = open('user_equip_location.pckl', 'wb')
pickle.dump(user, f)
f.close()