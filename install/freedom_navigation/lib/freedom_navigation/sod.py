import numpy as np
import NearestPoint

def sod(C, Q, dpkpc):
    n = C.shape[0]
    r, s = Q.shape
    y = dpkpc
    cntr = 0
    sum = 0
    for i in range(0, n-3, 3):
        cntr += 1
        # Loop to find distances from
        # data points to closest point
        # on the curve.
        for j in range(y[cntr-1], y[cntr]):
            np = NearestPoint(C[i:i+4,:].T, Q[:,j])
            d = Q[:,j] - np
            sum += np.dot(d, d) # Note: NearestPoint is a MATLAB
            if j == y[cntr-1] and i > 0: # interface program written by
                d2 = np.dot(d, d) # Dr C. Borges for some "C" rou-
                ds2 = np.dot(ds, ds) # tines obtained from 'Solving
                dm = max(d2, ds2) # the Nearest Point-on-Curve
                sum -= dm # Problem' and 'A Bezier Curve
                          # Root-Finder' developed P.J.
                          # Schneider in "Graphics Gems"
                          # Academic Press, 1990.
            ds = d
    return sum