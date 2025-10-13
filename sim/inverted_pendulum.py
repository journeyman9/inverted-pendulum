import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import yaml
from typing import Dict, List, Any, Optional
from simulation import Simulation
import scipy.integrate as spi
import datetime

class InvertedPendulum(Simulation):
    def __init__(self):
        super().__init__()
             
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"output_{timestamp_str}.csv"
    
    def inverted_pendulum_ode(self, t, x, u) -> np.ndarray:
        """
        x0: x
        x1: xd
        x2: th
        x3: thd
        """

        xd = np.zeros((len(x)))
        sigma = (1.0 - (self.Jm)/(self.r ** 2)) * (self.m1 + self.m2)
        
        xd[0] = x[2].copy()
        xd[1] = (self.m1 / sigma) * self.L * (x[3] ** 2) * np.sin(x[2]) - (self.m2 / sigma) * self.L * xd[3] * np.cos(x[2]) + (1.0 / sigma) * ((self.kt / (self.R * self.r)) * u - ((self.Bm / (self.r ** 2)) + ((self.kt * self.kb) / (self.R * self.r ** 2)) - self.b)) * x[1]
        xd[2] = x[3].copy()
        xd[3] = 0.5 * (self.m2 * self.g * self.L * np.sin(x[2])) / (self.I2 + self.m2 * self.L ** 2) - (self.m2 * self.L * xd[1] * np.cos(x[2])) / (self.I2 + self.m2 * self.L ** 2)
        
        return xd

    def reset(self, params) -> np.ndarray:
        self.t = 0
        self.dt = params["sim"]["dt"]
        self.ctr_dt = params["sim"]["ctr_dt"]
        self.time_end = params["sim"]["time_end"]

        self.m1 = params["mech"]["m1"]
        self.m2 = params["mech"]["m2"]
        self.m3 = params["mech"]["m3"]
        self.L = params["mech"]["L"]
        self.r = params["mech"]["r"]
        self.r3 = params["mech"]["r3"]
        self.b = params["mech"]["b"]
        self.Jm = params["mech"]["Jm"]
        self.g = params["mech"]["g"]
        
        self.kt = params["elec"]["kt"]
        self.R = params["elec"]["R"]
        self.kb = params["elec"]["kb"]
        self.Bm = params["elec"]["Bm"]
        
        self.I2 = 1/3 * self.m2 * self.L ** 2 + (2/5) * self.m3 * self.r3 ** 2 + self.m3 * self.L ** 2 
        
        self.state = np.array([
            0.01, # x
            0, # xdot
            np.radians(3), # th
            0 # thd
        ])
        return self.state
        
        
    def step(self, u) -> np.ndarray:
        def ode_wrapper(t, x):
            return self.inverted_pendulum_ode(t, x, u)

        sol = spi.solve_ivp(
            fun=ode_wrapper,
            t_span=(self.t, self.t + self.dt),
            y0=self.state,
            method='RK45',  # You can change this to 'BDF' if stiff
            max_step=self.dt
        )
        self.t = sol.t[-1]
        self.state = sol.y[:, 1]
        
        return np.array(self.state)
        
    def render(self):
        pass