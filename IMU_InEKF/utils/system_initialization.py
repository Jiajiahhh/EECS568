import numpy as np
import math
from functools import partial
from scipy.linalg import logm, expm

def wedge(phi):
    """
    R^3 vector to so(3) matrix
    @param  phi: R^3
    @return Phi: so(3) matrix
    """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

class myStruct:
    pass

def gfun(R, Omega, dt):
    R_pred = R @ expm(wedge(Omega) * dt)
    return R_pred

def Adj(X):
    return X

# JacobianMeasurementFunction
def Hfun(x):
    measurement = np.zeros((3, 3)) 
    measurement = wedge(x)
    return measurement

def system_initialization(alphas, beta):
    
    sys = myStruct()
    sys.Adj = Adj
    sys.gfun = gfun
    sys.Hfun = Hfun
    sys.V = 1e-4*np.eye(3)
    
    return sys