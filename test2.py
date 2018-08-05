# import numpy as np
# import pickle
# import matplotlib.pyplot as plt
# user = 5*np.random.random([400,2]) + np.array([3,3])
# plt.scatter(user[:,0], user[:,1])
# plt.show()
# f = open('user_equip_location.pckl', 'wb')
# pickle.dump(user, f)
# f.close()


# a=10
# b=0
# try:
#     c=a/b
#     print(c)
# except ZeroDivisionError as err:
#     print(err)
# print("done")


import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()
G.add_nodes_from([1,2,3])
G.add_edges_from([(1,2), (1,3)])
x = [i for i in nx.all_neighbors(G,1)]
G.remove_edges_from([(2,3)])

nx.draw(G)
plt.show()