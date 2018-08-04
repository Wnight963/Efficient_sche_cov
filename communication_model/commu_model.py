#######################################
#######################################
# this program computes commnication model of robot networks and display it.
import numpy as np
from cvxopt import matrix, solvers
from channel_capa import Capacity

N = 11
K = 1
source_data = 0.02
#this parameter needs to be small, otherwise it's hard to find optimal solution

def optimal_routing(x):

    'input: x: locations of robots; lead_index'
    'output: solution, routing strategy'

    ####################################### compute channel capacity

    R = np.zeros([N, N+K])
    for i in range(N):
        for j in range(N+K):
            R[i,j] = Capacity(x[i], x[j])

    #######################################inquality constraints
    G2 = np.eye(N)
    for i in range(N+K-1):
        G2 = np.hstack((G2,np.eye(N)))
    G11 = np.diag(R[:,0])
    for i in range(1, N+K):
        G11 = np.hstack((G11, np.diag(R[:, i])))
    G12 = np.zeros([N, N])
    G12[0,:] = R[:,0].T
    for i in range(1, N):
        tmp = np.zeros([N, N])
        tmp[i,:] = R[:, i].T
        G12 = np.hstack((G12, tmp))
    for i in range(K):
        G12 = np.hstack((G12, np.zeros([N, N])))
    G1 = G12 - G11
    G3 = -np.eye(N*(N+K))
    G = np.vstack((G1, G2, G3))

    h1 = -source_data * np.ones([N, 1])
    h2 = np.ones([N, 1])
    h3 = np.zeros([N*(N+K), 1])
    h = np.vstack((h1, h2, h3))

    #######################################objective function
    Q = np.eye(N*(N+K))
    p = np.zeros([N*(N+K), 1])

    #######################################sovle!
    # all the coefficients must be converted into cvxopt.matrix, np.array can't be
    # accepted by solver.qp
    Q = matrix(Q)
    p = matrix(p)
    G = matrix(G)
    h = matrix(h)

    solvers.options['show_progress'] = False
    solvers.options['maxiters'] = 100
    # solvers.options['reltol'] = 1e-2
    try:
        sol=solvers.qp(Q, p, G, h)
        T = np.array(sol['x'])
        T = T.reshape([N, N + K], order="F")
        if (sol['status'] == 'optimal'):
            return sol, T
        else:
            return sol, False
    except ValueError as err:
        print("Exception", err)
        res = {}
        res['status'] = 'nonoptimal'
        return res, False
    # T = np.array(sol['x'])
    # T = T.reshape([N, N+K], order="F")
    # if(sol['status']=='optimal'):
    #     return sol, T
    # else:
    #     return sol, False


def transmission(x, T):
    
    'return transmission data between robots, it is T time * channel capacity'

    R = np.zeros([N, N + K])
    # R is channel capacity matrix
    for i in range(N):
        for j in range(N + K):
            R[i, j] = Capacity(x[i], x[j])
    tmp = T * R
    return tmp

def ci(x, index, T):

    'test if robot i can send out more data than receive'
    'i.e., if the packet routing condition is obeyed'
    'x: robot locations; index: examined robot; T:routing strategy '

    R = np.zeros([N, N + K])
    for i in range(N):
        for j in range(N + K):
            R[i, j] = Capacity(x[i], x[j])
    tmp = T * R
    # transmission data amount
    ci = sum(tmp[index, :]) - sum(tmp[:, index]) - source_data
    return ci