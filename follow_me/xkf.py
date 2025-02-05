import numpy as np


class ExtendedKalmanFilter:

    def __init__(self, init_x: float, init_y: float, dt: float):
        self.dt: float = dt

        self.x: np.ndarray = np.array([[init_x], [init_y]])
        self.u: np.ndarray = np.zeros((2, 1), dtype=np.float64)
        
        self.A: np.ndarray = np.identity(2, dtype=np.float64) # 2x2 Identity Matrix
        self.B: np.ndarray = np.identity(2, dtype=np.float64) 
        self.C: np.ndarray = np.identity(2, dtype=np.float64)
        self.P: np.ndarray = np.identity(2, dtype=np.float64) * 0.1
        self.Q: np.ndarray = np.identity(2, dtype=np.float64)
        self.R: np.ndarray = np.identity(2, dtype=np.float64)

    
    def __del__(self):
        pass


    def prior_state_estimate(self) -> np.ndarray:
        """Mat[2,1]"""
        return self.A * self.x + self.B * self.u

    
    def prior_error_covariance_matrix(self) -> np.ndarray:
        """Mat[2,2]"""
        return self.A * self.P * self.A.T + self.Q


    def kalman_gain(self, _P: np.ndarray) -> np.ndarray:
        """Mat[2,2] -> Mat[2,2]"""
        return _P * self.C.T * np.linalg.inv(self.C * _P * self.C.T + self.R)


    def jacobian_matrix(self, dx: float, dy: float) -> None:
        self.A = np.array([[dx, 0.0], [0.0, dy]])
        return


    def state_estimate(self, x: np.ndarray, y: np.ndarray, kalman_gain: np.ndarray) -> np.ndarray:
        """Mat[2,1], Mat[2,1], Mat[2,2] -> Mat[2,1]"""
        return x + kalman_gain * (y - self.C * x)


    def posteriori_error_covariance_matrix(self, kalman_gain: np.ndarray, _P: np.ndarray) -> np.ndarray:
        """Mat[2,2], Mat[2,2] -> Mat[2,2]"""
        return (np.identity(2, dtype=np.float64) - kalman_gain * self.C) * _P


    def kalman_filter(self, _px: float, _py: float, _dx: float, _dy: float) -> tuple[float, float]:
        value_of_prior_state_estimate = self.prior_state_estimate()
        self.jacobian_matrix(_dx, _dy)
        value_of_prior_error_covariance_matrix = self.prior_error_covariance_matrix()
        kalman_gain_matrix = kalman_gain(value_of_prior_error_covariance_matrix)

        observation_matrix: np.ndarray = np.array([[_px], [_py]])
        
        value_state_estimate = self.state_estimate(observation_matrix, value_of_prior_state_estimate, kalman_gain_matrix)
        value_posteriori_error_covariance_matrix = self.posteriori_error_covariance_matrix(kalman_gain_matrix, value_of_prior_error_covariance_matrix)

        self.x = value_state_estimate
        self.P = value_posteriori_error_covariance_matrix

        return (value_state_estimate[0][0], value_state_estimate[1][0])





