import numpy as np
from opdist import opdist
from scipy.optimize import fmin, minimize

def bstdst(id, Q, P, ang, k):
    options = {'disp': False, 'xtol': 0.01, 'ftol': 0.01}
    n = len(id)
    bdt = np.empty((id.shape[0], n))
    for i in range(n):
        # bdt[:, i] = fmin(opdist, id[:, [i - 1]], args=(Q[:, k[i]-1:k[i+1]], P[:, i:i+2], ang[i:i+2]), **options)
        bdt[:, i] = fmin(opdist, id[:, [i - 1]], args=(Q[:, k[i]-1:k[i+1]], P[:, i:i+2], ang[i:i+2]), **options)
        result = minimize(opdist, id[:, [i - 1]], args=(Q[:, k[i]-1:k[i+1]], P[:, i:i+2], ang[i:i+2]), method='Nelder-Mead', options=options)
        # bdt[:,i] = result.x
        # print("bdt[:, i]", bdt[:, i])
        # print("Q[:, k[i]-1:k[i+1]-1]: ", Q[:, k[i]-1:k[i+1]])
        # print("id[:, [i]]: ", id[:, [i]]) # Valor certo
        # print("P[:, i:i+2]: ", P[:, i:i+2]) # Valor certo
        # print("ang[i:i+2]: ", ang[i:i+2]) # Valor certo
    print("bdt: ", bdt)
    return bdt