import numpy as np
import matplotlib.pyplot as plt

from defk import defk
from pltC import pltC
from tang import tang
from ctpts import ctpts
from knots import knots
from distEJL import distEJL

def iguess0(Q, n, k):
    r, m = Q.shape

    # Q = datapoints (2xm)
    # n = The number of knotpoints
    # k = default knot positions (indices 1...m)
    if k is None:
        k = defk(m, n)  # Calls for default knot position.
    dpkpc = k  # Position of knot points passed globally.
    P = knots(Q, k)  # call to compute the knotpoints.
    dt = distEJL(P, n)  # Call to compute the distance between successive knot points.
    ang = tang(Q, k)  # Call to compute the angles for the unit tangent vectors.
    C = ctpts(P, ang, dt)  # Call to compute the control points for the curve.
    CT = C.T

    unique_control = CT[~np.isin(CT, P).all(axis=1)]

    # Manter o segundo e penúltimo ponto
    ctrl_pts = np.concatenate((unique_control[1:2], unique_control[-2:-1]))

    # Removendo os pontos comuns
    # Removendo o primeiro e último ponto
    ctrl_pts = np.array(unique_control)

    # pltC(C, Q, P)  # Call to plot the initial guess curve, its control polygon, and points in Q.
    # plt.legend()
    # plt.gcf()
    # plt.title('Adequação da curva de Bézier')
    # plt.show()
    return ctrl_pts