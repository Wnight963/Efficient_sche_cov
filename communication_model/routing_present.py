import numpy as np
import matplotlib.pyplot as plt
import datetime

def routing_graph(x, transmission, N, K):

    'input: x:locations of robots'

    plt.figure("routing")
    plt.scatter(x[:,0],x[:,1])
    plt.scatter(5.6, 3.0)
    n = range(N+K)
    for i,txt in enumerate(n):
        plt.annotate(txt, (x[i,0],x[i,1]))             # number the points
    # plt.annotate('leader', x[leader_index])
    for i in range(N):
        for j in range(N+K):
            if transmission[i,j]>0:
                a = x[[i,j],0]
                b = x[[i,j],1]
                plt.plot(a, b, linewidth=50*transmission[i,j])       # thickness denotes transmmision rate
    plt.ion()
    plt.pause(0.2)
    plt.close()
    # plt.show()

    # now_time = datetime.datetime.now()
    # name = datetime.datetime.strftime(now_time,'%Y-%m-%d %H_%M_%S')
    # plt.savefig('picture/'+name)
    return