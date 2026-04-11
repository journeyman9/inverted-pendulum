import numpy as np
import yaml

class Controller():
    def __init__(self):
        with open('params.yaml', 'r') as cfg:
            params = yaml.safe_load(cfg)
        self.K = np.array(params["ctrl"]["K"])
    
    def lqr(self, x: np.ndarray, x_r: np.ndarray) -> np.ndarray:
        # Standard state-feedback law from the LQR design.
        x_tilde = x[0] - 0 # In sim we start at 0, but in hardware this may not be true
        theta_tilde = x[2] - np.pi
        
        error = np.array([
            x_tilde - x_r[0],
            x[1] - x_r[1],
            theta_tilde - x_r[2],
            x[3] - x_r[3]
        ])

        u = -self.K.dot(error)

        return float(u)