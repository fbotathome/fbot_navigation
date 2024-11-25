from ctpts import ctpts
from typing import List
from NearestPoint import NearestPoint

def opdist(id, Q, P, ang):
    import numpy as np
    """
    This function is the objective function to be minimized during segment optimization.
    It receives subcomponents from a curve's composite vector, a segment at a time.
    It returns the sum error for a segment.
    """
    n = len(Q[0])
    # print("Q[0]:", Q[0])
    # print("Q:", Q)
    # print("n: ", n)
    id = np.reshape(id, (-1, 1))

    # print("----")
    # print("id: ", id)
    # print("Q: ", Q)
    # print("P: ", P)
    # print("ang: ", ang)

    C = ctpts(P, ang, id) # Call to compute the segments control points.
    # print("C: ", C)
    se2 = 0
    cont = 0
    
    # PROBLEMA DAQUI PRA BAIXO

    # Loop to find distance error in a segment.
    for j in range(n):
        # print("j: ", j)
        npoints = NearestPoint(C.T, np.transpose((Q[:,j])))
        # print("npoints: ", npoints)
        # print("np.transpose(tuple(C[:, 0])): ", np.transpose(tuple(C[:, 0])))
        if (j == 0) and np.all(npoints == np.transpose(tuple(C[:, 0]))):
            # print("Entrou 1")
            d = np.zeros(2)
        elif (j == n - 1) and np.all(npoints == np.transpose(tuple(C[:, 3]))):
            # print("Entrou 2")
            d = np.zeros(2)
        else:
            # print("Entrou 3")
            d = np.transpose(((Q[:,j])) - npoints)
        # print("d: ", d)
        # print("se2: ", se2)

        se2 = se2 + np.dot(d, d.T)
        # print("se2: ", se2)
        cont += 1
    return se2