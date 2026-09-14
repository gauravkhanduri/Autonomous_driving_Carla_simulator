import numpy as np

from ads_pipeline.kalman_filter import KalmanFilter


def test_constant_velocity_prediction():
    kf = KalmanFilter(dt=0.5)
    kf.x = np.array([1.0, 2.0, 2.0, -1.0])
    kf.predict()
    assert np.allclose(kf.x, [2.0, 1.5, 2.0, -1.0])


def test_converges_to_moving_target_velocity():
    rng = np.random.default_rng(0)
    kf = KalmanFilter(dt=0.1, meas_std=0.2)
    for k in range(200):
        t = k * 0.1
        truth = np.array([5.0 + 3.0 * t, -1.0 + 0.5 * t])
        kf.predict()
        kf.update(truth + rng.normal(0.0, 0.2, 2))
    assert np.allclose(kf.x[2:], [3.0, 0.5], atol=0.3)
    assert np.allclose(kf.P, kf.P.T)
    assert np.all(np.linalg.eigvalsh(kf.P) > 0)


def test_lower_measurement_noise_pulls_estimate_harder():
    precise, noisy = KalmanFilter(), KalmanFilter()
    for kf in (precise, noisy):
        kf.P = np.eye(4)
    precise.update([1.0, 0.0], R=np.eye(2) * 0.01)
    noisy.update([1.0, 0.0], R=np.eye(2) * 10.0)
    assert precise.x[0] > noisy.x[0] > 0.0
