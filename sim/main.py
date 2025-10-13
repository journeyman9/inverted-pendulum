import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from inverted_pendulum import InvertedPendulum
from factory import Factory
from planner import Planner
from controller import Controller
import yaml

def main():
    with open('params.yaml', 'r') as cfg:
        params = yaml.safe_load(cfg)
        
    factory  = Factory()
    factory.register("ip", InvertedPendulum)
    sim = factory.create("ip")
    
    x = sim.reset(params)

    steps_per_ctrl = int(sim.ctr_dt / sim.dt)
    total_steps = int(sim.time_end / sim.dt)
    
    planner = Planner()
    x_r = planner.plan(x)
    
    ctr = Controller()
    u = 0

    done = False
    i = 0
    while not done:
        sim.log(i, x, x_r, u)
        if i % steps_per_ctrl == 0:
            x_r = planner.plan(x)
            u = ctr.lqr(x, x_r)
            
        x = sim.step(u)
        
        if i % 100 == 0:
            #sim.render()
            print("Time : {} s".format(sim.dt * i))
        
        if i >= total_steps-1:
            done = True

        i += 1
        
    print("Simulation finished.")

if __name__ == "__main__":
    main()
