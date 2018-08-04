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


# import numpy as np
# import pickle
# import matplotlib.pyplot as plt
#
# user = 2*np.random.random([400, 2]) +np.array([3,2])
# plt.scatter(user[:,0], user[:,1], linewidths=1)
# plt.grid(True)
# plt.show()
# print(user.shape)
# f = open('user_equip_location.pckl', 'wb')
# pickle.dump(user, f)
# f.close()

a = [1,2]
b = [[1,2], [3,4]]
print(type(a[0]))
print(type(b[0])==list)
