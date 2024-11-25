from cubicBezierToPolyline import cubicBezierToPolyline
from findClosestPoint import findClosestPoint

def NearestPoint(C, P):
    # Compute complete Bezier
    # print("------")
    # print("C: ", C)
    # print("P: ", P)
    Q = cubicBezierToPolyline(C, 128)
    # print("Q: ", Q.shape) # (128, 2)

    # Find nearest point on Bezier
    ind = findClosestPoint(P, Q)
    ind = ind[0]
    ind = int(ind[0])
    # print("ind: ", ind) # Vai até 126, deveria ir até 127

    np = Q[ind]
    # print("np: ", np)
    return np