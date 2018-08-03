import numpy as np
import pickle

x = np.array([[1,2], [3,4], [5,6]])
user = np.array([1, 1])

print(abs(user)<x[2])

def if_is_detected(x, user):
    distance = x - user
    photo_range = np.array([1,2])
    dectected = [list(abs(n)<photo_range) for n in distance]
    res = [True, True] in dectected
    return res

print(if_is_detected(x, user))

print(user not in x)

