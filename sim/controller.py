import numpy as np
import yaml

class Controller():
    def __init__(self):
        with open('params.yaml', 'r') as cfg:
            params = yaml.safe_load(cfg)
        self.K = np.array(params["ctrl"]["K"])
    
    def lqr(self, x: np.ndarray, x_r: np.ndarray) -> np.ndarray:
        # Standard state-feedback law from the LQR design.
        error = x - x_r
        u = -self.K.dot(error)

        return float(u)