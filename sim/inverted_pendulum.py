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
        sigma = 1 + self.Jm / (self.r ** 2 * (self.m1 + self.m2)) - (self.m2 * self.L) ** 2 / ((self.m1 + self.m2) * (self.I2 + self.m2 * self.L ** 2))
        
        xd[0] = x[1]
        xd[2] = x[3]
        
        # Solve the coupled system for xd[1] and xd[3]
        # From the equations:
        # xd[1] = term1 - (m2/sigma) * L * xd[3] * cos(th) + term2 * u - damping * x[1]
        # xd[3] = term3 - (m2 * L * xd[1] * cos(th)) / (I2 + m2*L^2)
        
        # Define coefficients
        cos_th = np.cos(x[2])
        sin_th = np.sin(x[2])
        
        term1 = (self.m1 / sigma) * self.L * (x[3] ** 2) * sin_th
        term2 = (1.0 / sigma) * (self.kt / ((self.m1 + self.m2) * self.R * self.r))
        damping_coeff = (1.0 / sigma) * ((self.Bm / (self.r ** 2)) + ((self.kt * self.kb) / (self.R * self.r ** 2)) - self.b)
        term3 = 0.5 * (self.m2 * self.g * self.L * sin_th) / (self.I2 + self.m2 * self.L ** 2)
        
        # Coupling coefficients
        A_cart = (self.m2 / sigma) * self.L * cos_th
        A_pend = (self.m2 * self.L * cos_th) / (self.I2 + self.m2 * self.L ** 2)
        
        # Solve the coupled system algebraically
        # xd[1] = term1 - A_cart * xd[3] + term2 * u - damping_coeff * x[1]
        # xd[3] = term3 - A_pend * xd[1]
        #
        # Substituting: xd[3] = term3 - A_pend * (term1 - A_cart * xd[3] + term2 * u - damping_coeff * x[1])
        # xd[3] = term3 - A_pend*term1 + A_pend*A_cart*xd[3] - A_pend*term2*u + A_pend*damping_coeff*x[1]
        # xd[3] * (1 - A_pend*A_cart) = term3 - A_pend*term1 - A_pend*term2*u + A_pend*damping_coeff*x[1]
        
        denom = 1 - A_pend * A_cart
        
        if abs(denom) < 1e-10:
            # Fallback if denominator is too small
            xd[3] = term3
            xd[1] = term1 + term2 * u - damping_coeff * x[1]
        else:
            xd[3] = (term3 - A_pend * term1 - A_pend * term2 * u + A_pend * damping_coeff * x[1]) / denom
            xd[1] = term1 - A_cart * xd[3] + term2 * u - damping_coeff * x[1]
        
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
            params["init"]["x"],
            params["init"]["xd"],
            np.radians(params["init"]["th"]) + np.pi,
            params["init"]["thd"],
        ])
        self.u = 0
        return self.state
        
    def step(self, u) -> np.ndarray:
        self.u = u
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
        # Initialize figure on first call
        if not hasattr(self, 'fig') or self.fig is None:
            plt.ion()  # Turn on interactive mode
            self.fig, self.ax = plt.subplots(figsize=(14, 8))
            self.ax.set_xlim(-0.8, 0.8)
            self.ax.set_ylim(-0.2, 0.8)
            self.ax.set_aspect('equal')
            self.ax.set_title('Inverted Pendulum Simulation', fontsize=14, fontweight='bold')
            self.ax.set_xlabel('Position (m)', fontsize=12)
            self.ax.set_ylabel('Height (m)', fontsize=12)
        
        # Clear previous frame
        self.ax.clear()
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.2, 0.8)
        self.ax.set_aspect('equal')
        self.ax.set_title(f'Inverted Pendulum Simulation - Time: {self.t:.3f}s', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('Position (m)', fontsize=12)
        self.ax.set_ylabel('Height (m)', fontsize=12)
        
        # Extract state variables
        x_cart = self.state[0]  # cart position
        th_phys = self.state[2] # physical angle, equilibrium at pi
        th = th_phys - np.pi    # Deviation angle: 0 = upright
        
        # Track/rail position
        track_y = 0.0
        track_length = 1.6
        
        # Draw track/rail
        self.ax.plot([-track_length/2, track_length/2], [track_y, track_y], 
                    'k-', linewidth=4, label='Track', zorder=1)
        
        # Draw pulley on the right side (radius multiplied by 2 for diameter)
        pulley_center_x = 0.6
        pulley_radius = self.r  # Use for visual radius
        pulley_center_y = track_y + pulley_radius
        pulley_circle = plt.Circle((pulley_center_x, pulley_center_y), pulley_radius, 
                                color='#666666', fill=True, edgecolor='black', 
                                linewidth=2, zorder=3)
        self.ax.add_patch(pulley_circle)
        # Draw pulley center and mounting
        self.ax.plot(pulley_center_x, pulley_center_y, 'ko', markersize=6, zorder=4)
        # Draw pulley mounting bracket
        self.ax.plot([pulley_center_x, pulley_center_x], [pulley_center_y, track_y], 
                    'k-', linewidth=2, zorder=2)
        
        # Draw belt connecting pulley to cart (horizontal)
        belt_attachment_x = x_cart + 0.08  # Right edge of cart
        belt_y = track_y + 2*pulley_radius  # Horizontal belt height
        belt_end_x = pulley_center_x # Left edge of pulley
        
        self.ax.plot([belt_attachment_x, belt_end_x], [belt_y, belt_y], 
                    'brown', linewidth=3, linestyle='-', alpha=0.8, label='Belt', zorder=2)
        
        # Draw cart (m1) - rectangle on track
        cart_width = 0.12
        cart_height = 0.08
        cart_x = x_cart - cart_width/2
        cart_y = track_y
        
        cart_rect = plt.Rectangle((cart_x, cart_y), cart_width, cart_height,
                                facecolor='#4A90E2', edgecolor='black', 
                                linewidth=2, zorder=3)
        self.ax.add_patch(cart_rect)
        
        # Draw wheels on cart
        wheel_radius = 0.02
        wheel1_x = cart_x + cart_width * 0.25
        wheel2_x = cart_x + cart_width * 0.75
        wheel_y = track_y - wheel_radius
        
        wheel1 = plt.Circle((wheel1_x, wheel_y), wheel_radius, 
                        color='black', fill=True, zorder=3)
        wheel2 = plt.Circle((wheel2_x, wheel_y), wheel_radius, 
                        color='black', fill=True, zorder=3)
        self.ax.add_patch(wheel1)
        self.ax.add_patch(wheel2)
        
        # Label cart mass
        self.ax.text(x_cart, cart_y + cart_height/2, f'm1={self.m1:.3f}kg', 
                    ha='center', va='center', fontsize=9, weight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
        
        # Pendulum pivot point (top center of cart)
        pivot_x = x_cart
        pivot_y = cart_y + cart_height
        
        # Calculate pendulum end point
        pendulum_end_x = pivot_x + self.L * np.sin(th)
        pendulum_end_y = pivot_y + self.L * np.cos(th)
        
        # Draw pendulum rod (m2)
        self.ax.plot([pivot_x, pendulum_end_x], [pivot_y, pendulum_end_y],
                    color='#2ECC71', linewidth=5, label='Pendulum Rod (m2)', zorder=3)
        
        # Draw circular mass (m3) at the end of pendulum
        mass_circle = plt.Circle((pendulum_end_x, pendulum_end_y), self.r3,
                                color='#E74C3C', fill=True, edgecolor='black',
                                linewidth=2, zorder=4)
        self.ax.add_patch(mass_circle)
        
        # Draw pivot point
        self.ax.plot(pivot_x, pivot_y, 'ko', markersize=10, zorder=5)
        pivot_circle = plt.Circle((pivot_x, pivot_y), 0.015, 
                                color='black', fill=True, zorder=5)
        self.ax.add_patch(pivot_circle)
        
        # Add parameter info box
        info_text = f'Parameters:\n'
        info_text += f'L = {self.L:.3f} m\n'
        info_text += f'r (pulley) = {self.r:.3f} m\n'
        info_text += f'\nState:\n'
        info_text += f'x = {x_cart:.3f} m\n'
        info_text += f'ẋ = {self.state[1]:.3f} m/s\n'
        info_text += f'θ = {np.degrees(th):.2f}°\n'
        info_text += f'θ̇ = {np.degrees(self.state[3]):.2f}°/s'
        info_text += f'\nAction:\n'
        info_text += f'u = {self.u:.2f}'
        
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                    fontsize=9, verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='wheat', 
                    alpha=0.9, edgecolor='black', linewidth=1))
        
        # Add legend
        self.ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
        
        # Draw reference line (vertical) to show ideal inverted position
        self.ax.plot([pivot_x, pivot_x], [pivot_y, pivot_y + self.L], 
                    'r--', linewidth=1, alpha=0.3, label='Reference (θ=0°)')
        
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # Small pause to allow rendering