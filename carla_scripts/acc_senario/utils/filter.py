# Revised Kalman Filter Implementation

import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterM:
    def __init__(self, x_init, F, H, P, R, Q):
        dim_x = x_init.shape[0]
        dim_z = H.shape[0]
        self.kf = KalmanFilter(dim_x, dim_z)
        # self.kf = KalmanFilter(dim_x=x_init.shape[0], dim_z=H.shape[0])
        self.kf.x = np.array(x_init)
        self.kf.F = np.array(F)
        self.kf.H = np.array(H)
        self.kf.P = np.array(P)
        self.kf.R = np.array(R)
        self.kf.Q = np.array(Q)
    
    def predict(self, F=None):
        if F is not None:
            self.kf.F = F
        self.kf.predict()
    
    def update(self, z):
        self.kf.update(z)

    @property
    def state(self):
        return [float(self.kf.x[0]),float(self.kf.x[1])]
    
    @property
    def acceleration(self):
        return self.kf.x[0]
    
    @property
    def velocity(self):
        return self.kf.x[1]
    
class LowPassFilter:
    def __init__(self, alpha = 0.6):
        self.alpha = alpha
        self.x = None

    def update(self, measurement):
        if self.x is None:
            self.x = measurement
        else:
            self.x = self.alpha * measurement + (1 - self.alpha) * self.x

    @property
    def state(self):
        return self.x

# import imu class from carla
class CarlaIMULowPassFilter(LowPassFilter):
    def __init__(self, alpha = 0.8):
        super().__init__(alpha)

    def update(self, measurement):
        measurement = np.array(measurement)
        super().update(measurement)