import numpy as np
from scipy.optimize import minimize
from objf2 import objf2

def globop(xi, Q, t, k, dpkpc):
    options = {'disp': False, 'xtol': 0.01, 'ftol': 0.01}
    GOC = minimize(objf2, xi, args=(Q, t, k, dpkpc), method='Nelder-Mead', options=options)
    return GOC.x
