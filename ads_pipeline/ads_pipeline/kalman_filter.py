import numpy as np


class KalmanFilter:
    """Linear constant-velocity Kalman filter for a 2-D target."""

    def __init__(self, dt=0.1, accel_std=0.5, meas_std=0.5):

        # state vector : [x,y,vx,vy]
        self.x = np.zeros(4)

        # state covariance
        self.P = np.eye(4) * 100.0

        # measurement matrix model : observe position only
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        # default measurement noise
        self.R = np.eye(2) * meas_std**2

        # acceleration noise std
        self.accel_std = accel_std

        self.set_dt(dt)

    def set_dt(self, dt):
        self.dt = dt

        # state Transition (constant velocity)
        self.F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Process noise: discrete white-noise acceleration model
        G = np.array([
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt]
        ])
        self.Q = G @ G.T * self.accel_std**2

    def predict(self, dt=None):
        if dt is not None and dt != self.dt:
            self.set_dt(dt)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, z, R=None):
        R = self.R if R is None else R
        y = np.asarray(z, dtype=float) - self.H @ self.x   # innovation
        S = self.H @ self.P @ self.H.T + R                  # innovation covariance
        K = np.linalg.solve(S, self.H @ self.P).T           # Kalman gain (P H^T S^-1; P, S symmetric)
        self.x = self.x + K @ y
        # Joseph form keeps P symmetric positive definite
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return self.x
