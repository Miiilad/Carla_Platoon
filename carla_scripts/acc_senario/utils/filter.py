# Revised Kalman Filter Implementation

import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, x0, P0, Q0, R0):
        self.F = F
        self.B = B
        self.H = H
        self.x = x0
        self.P = P0
        self.P_p = P0
        self.Q = Q0
        self.R = R0

    def time_update(self, control=None, F=None, B=None):
        if F is not None:
            self.F = F
        if B is not None:
            self.B = B

        if control is not None:
            self.x = self.F @ self.x + self.B @ control
        else:
            self.x = self.F @ self.x

        self.P_p = self.F @ self.P @ self.F.T + self.Q

    def measurement_update(self, measurement):
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P_p @ self.H.T @ np.linalg.inv(S)
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P_p
        self.x = self.x + K @ y

    @property
    def state(self):
        return self.x

# the input of this filter is position
class AccSpeedPosKF(KalmanFilter):
    def __init__(self, time_interval_approx, x0, P0, Q0, R0):
        dt = time_interval_approx
        F = np.array([[1, dt, 0.5 * dt**2], [0, 1, dt], [0, 0, 0]])
        B = np.array([[0], [0], [1]])
        H = np.array([[1, 0, 0]])
        super().__init__(F, B, H, x0, P0, Q0, R0)

    def time_update(self, control=None, time_interval_approx=None):
        if time_interval_approx is not None:
            self.F = np.array([[1, time_interval_approx, 0.5 * time_interval_approx**2], [0, 1, time_interval_approx], [0, 0, 1]])
        super().time_update(control, self.F, self.B)

    def measurement_update(self, measurement):
        super().measurement_update(measurement)

    @property
    def velocity(self):
        return self.state[1][0]
    
    @property
    def position(self):
        return self.state[0][0]
    
    @property
    def acceleration(self):
        return self.state[2][0]



# >>>>>>>>>>>>>>>>>>>>>>>>>>>>> TEST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
def test_kalman_filter():
    time_interval_approx = 0.01
    times = np.arange(0.0, 10.0, time_interval_approx)
    
    measurement_noise = 10.0

    pos = 0.0
    vel = 0.0
    acc = 0.0

    x0 = np.array([[0], [0], [0]])
    P0 = np.eye(3) 
    Q0 = np.eye(3) * 0.1
    R0 = np.eye(1) * measurement_noise

    kf = AccSpeedPosKF(time_interval_approx, x0, P0, Q0, R0)
    
    pos_record = []
    pos_noise_record = []
    ground_truth_record = []
    vel_record = []
    
    prev_time = 0.0
    # velocity_profile = 20 * np.sin(0.2 * np.pi * times)
    acc = 9.8

    for i, time in enumerate(times): 
        dt = time - prev_time
        prev_time = time
        
        vel+= acc * dt
        pos+= vel * dt

        pos_noise = pos + np.random.normal(0, measurement_noise)
        kf.time_update(np.array([[acc]]))
        kf.measurement_update(pos_noise)

        pos_record.append(kf.position)
        pos_noise_record.append(pos_noise)
        ground_truth_record.append(pos)
        vel_record.append(kf.velocity)


    USE_PLOTLY = True

    if not USE_PLOTLY:
        import matplotlib.pyplot as plt
        plt.subplot(2, 1, 1)
        plt.plot(times, pos_record, label='Kalman Filter')
        plt.scatter(times, pos_noise_record, label='Measurement', c='r', s=1)
        plt.plot(times, ground_truth_record, label='Ground Truth')
        plt.legend()
        plt.ylabel('Position')
        plt.xlabel('Time')
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(times, vel_record, label='Kalman Filter')
        plt.legend()
        plt.ylabel('Velocity')
        plt.xlabel('Time')
        plt.grid(True)

        plt.show()
        
    # subplots
    if USE_PLOTLY:
        import plotly.graph_objects as go
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=times, y=pos_record, mode='lines', name='Kalman Filter'))
        fig.add_trace(go.Scatter(x=times, y=pos_noise_record, mode='markers', name='Measurement', marker=dict(size=1, color='red')))
        fig.add_trace(go.Scatter(x=times, y=ground_truth_record, mode='lines', name='Ground Truth'))
        fig.update_layout(title='Position', xaxis_title='Time', yaxis_title='Position')
        fig.show()


if __name__ == '__main__':
    test_kalman_filter()
