import numpy as np
from scipy.linalg import block_diag
from scipy.linalg import logm, expm

def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    rotation_matrix = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
                                 [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                                 [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]])

    yaw = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
    return yaw

def quaternion_to_rotation_matrix(quat):
    """
    Convert a quaternion into a rotation matrix.
    Args:
    quat (np.array): Quaternion [w, x, y, z]
    """
    x, y, z, w = quat
    R = np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]
    ])
    return R



class InEKF:

    def __init__(self, system):
        # Right_IEKF  
        self.Adj = system.Adj
        self.Phi = np.eye(3)
        self.gfun = system.gfun
        self.V = system.V # measurement noise covariance
        self.Hfun = system.Hfun
        self.Sigma = 0.001 * np.eye(3)
        self.P = 0.1 * np.eye(3)  # state covariance
        self.Q = 1e-4*np.eye(3)

    def prediction(self, data, ang, i, R):
        
        if i == 0:
            R_pred = R
        else:
            R_pred = self.mu_pred

        X =  self.gfun(R_pred, ang, data[i][3])
        Ad_x = self.Adj(X)

        P = Ad_x @ self.Q @ Ad_x.T + self.P

        return X, P
    
    def correction(self, data, mu_pred, sigma_pred, acc, i):

        self.mu_pred = mu_pred
        self.sigma_pred = sigma_pred

        g = np.array([0, 0, 9.81]).reshape(-1,1)

        H = np.zeros((3,3))

        H = self.Hfun(g)

        N = self.mu_pred @ self.V @ self.mu_pred.T

        S = H @ self.sigma_pred @ H.T + N

        L = self.sigma_pred @ H.T @ np.linalg.inv(S)
        
        nu = (self.mu_pred @ acc).reshape(-1,1) - g
        
        delta = self.Hfun(L @ nu)

        self.mu_pred = expm(delta) @ self.mu_pred

        X = np.array([data[i][0], data[i][1], 0]).reshape(3,1)
        X = self.mu_pred @ X 
        if X[0] <= 0:
            X[1] = -X[1] +2

        I = np.eye(np.shape(self.sigma_pred)[0])

        temp = I - np.dot(L, H)

        self.Sigma = np.dot(np.dot(temp, self.sigma_pred), temp.T) + np.dot(np.dot(L, N), L.T)

        return np.copy(X), np.copy(self.Sigma)
