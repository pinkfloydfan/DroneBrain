import numpy as np

#linear kalman filter for a 1D problem
class Kalman_1D:

    def __init__(self, Q, R, x_initial, dt): 
        
        #variable we're tracing
        self.x_hat = x_initial
        self.x_hat_dot = 0

        self.state = np.array([[x_hat],[x_hat_dot]])
        
        #state covariance
        self.sigma_x     = 0.5
        self.sigma_x_dot = 0.5

        #process noise
        self.Q = Q
        
        #measurement noise
        self.R = R

        self.A = np.array([[1, dt], [0, x]],np.single)
        self.B = np.array([[0.5*dt^2],[dt]])
        self.C = np.array([1, 0])


        self.P = np.array([[sigma_x, 0], [0, sigma_x_dot]])

    def predictState(self, u):

        x_hat_old = self.x_hat

        self.x_hat = A.dot(x_hat_old) + B.dot(u)
        self.P = (np.transpose(self.A).dot(self.P)).dot(self.A)

        return self.x_hat

    def correct(self, z): 

        C_T = np.transpose(self.C)

        den = np.linalg.inv((self.C.dot(self.P).dot(C_T) + self.R))

        G = P.dot(C_T).dot(den)

        x_hat_old = self.x_hat

        self.x_hat = x_hat_old + G.dot(z-self.C.dot(x_hat_old))

        p_old = self.P

        self.P = (np.eye(2)-G.dot(self.C)).dot(p_old)

        return self.x_hat

        
        