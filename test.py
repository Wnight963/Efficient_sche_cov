import pickle
from robot_class import ROBOT

f = open('store.pckl', 'rb')
obj = pickle.load(f)
f.close()

print(obj)
for x in obj:
    print(x.number, x.role, x.location)