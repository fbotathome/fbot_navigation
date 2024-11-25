import numpy as np
from ktangdt import ktangdt
from bstdst import bstdst

def segop(k, Q, x0):
    # Separates the vector x0 into its subcomponents.
    # print("k: ", k)
    # print("x0: ", x0)

    P, ang, dt = ktangdt(x0)

    print("P: ", P)
    print("ang: ", ang)
    print("dt: ", dt)
    print("Q: ", Q)
    # Call to the function which finds the optimum distances for a segment.
    bdt = bstdst(dt, Q, P, ang, k)

    print("bdt: ", bdt)
    bdt1 = [] # Empty list
    # PROBLEMA TA ACIMA DAQUI bdt VEM ERRADO

    for i in range(2):
        bdt1.extend(bdt[i, :])

    # Assemble the vector of parameters for the curve.
    SOC = np.concatenate([P[0, :], P[1, :], ang, bdt1])
    return SOC