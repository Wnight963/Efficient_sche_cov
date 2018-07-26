# this script calculates link capacity between two robots
import numpy.linalg as LA
import numpy as np
u = 0.4
l = 2

def Capacity(x,y):

    'input: two locations'
    'output: channel capacity between them'

    distance = LA.norm(x - y)
    if distance>l:
        return 0
    elif distance<u:
        return 1
    else:
        a = -5/8 * distance + 5/4
        return a


