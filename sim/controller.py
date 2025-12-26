import numpy as np
import yaml

class Controller():
    def __init__(self):
        with open('params.yaml', 'r') as cfg:
            params = yaml.safe_load(cfg)
        self.K = np.array(params["ctrl"]["K"])
    
    def lqr(self, x: np.ndarray, x_r: np.ndarray) -> np.ndarray:
        # Work in deviation-from-upright coordinates
        x_tilde   = x.copy()
        x_r_tilde = x_r.copy()

        x_tilde[2]   = x_tilde[2]   - np.pi  # θ̃ = θ - π
        x_r_tilde[2] = x_r_tilde[2] - np.pi  # θ̃_ref = θ_ref - π (→ 0)

        u = self.K.dot(x_tilde - x_r_tilde)

        # NOTE: Voltage sign convention correction
        # The controller gains were designed assuming positive voltage moves cart right,
        # but the physical system (or motor wiring) has reversed polarity.
        # Negating the action compensates for this sign mismatch.
        u *= -1
        return float(u)