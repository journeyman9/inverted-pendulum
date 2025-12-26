from abc import ABC, abstractmethod
import numpy as np
import pandas as pd
from typing import Dict, List
import datetime

class Simulation(ABC):
    @abstractmethod
    def reset(self, params: Dict) -> np.ndarray:
        pass
    
    @abstractmethod
    def render(self):
        pass

    @abstractmethod
    def step(self, u) -> np.ndarray:
        pass
    
    def log(self, iteration: int, x: np.ndarray, x_r: np.ndarray, u: float, time: float = None):
        data = {}

        if time is not None:
            data["time"] = time
        
        for i in range(len(x)):
            data[f"x{i}"] = x[i]
            
        for i in range(len(x_r)):
            data[f"x_r{i}"] = x_r[i]
        
        data["u"] = u

        df = pd.DataFrame([data])
        
        if iteration == 0:
            df.to_csv(self.filename, index=False, mode='w')
        else:
            df.to_csv(self.filename, index=False, mode='a', header=False)