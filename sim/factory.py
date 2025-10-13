from typing import Dict, List, Any
from simulation import Simulation

class Factory(): 
    def __init__(self):
        self.registry = {}

    def register(self, sim_type: str, clss: Simulation):
        self.registry.update({sim_type: clss})
    
    def create(self, sim_type: str) -> Simulation:
        return self.registry[sim_type]()
    