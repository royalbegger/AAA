import numpy as np
import time

class KalmanTrackerCA:

    def __init__(self, init_pos, dt=0.1):

        self.dt = dt

        # 状态: px py vx vy ax ay
        self.x = np.zeros((6,1))
        self.x[0,0] = init_pos[0]
        self.x[1,0] = init_pos[1]

        self.P = np.eye(6) * 10.0

        self.Q = np.eye(6) * 0.1
        self.R = np.eye(2) * 0.2

        self.H = np.zeros((2,6))
        self.H[0,0] = 1
        self.H[1,1] = 1

        self.last_update = time.time()
        self.missed = 0
        self.age = 0

    def predict(self):

        dt = self.dt

        F = np.eye(6)
        F[0,2] = dt
        F[1,3] = dt
        F[0,4] = 0.5*dt*dt
        F[1,5] = 0.5*dt*dt
        F[2,4] = dt
        F[3,5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        self.age += 1
        self.missed += 1

    def update(self, z):

        z = np.array(z).reshape(2,1)

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        self.missed = 0

    def get_state(self):
        return self.x.flatten()