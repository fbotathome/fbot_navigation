import numpy as np
import sys

print(sys.path)

from unitv import unitv

def tang(Q, k):
    u = unitv(Q, k)
    ang = np.arctan2(u[1,:], u[0,:])
    return ang
