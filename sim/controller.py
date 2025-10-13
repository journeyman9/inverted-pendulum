import numpy as np

class Controller():
    def __init__(self):
        pass
    
    def lqr(self, x: np.ndarray, x_r: np.ndarray) -> np.ndarray:
        K = np.array([1., 0.95425908, -1.00477956, -1.06011656])
        u = K.dot(x - x_r)
        return u