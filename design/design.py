import numpy as np
import matplotlib.pyplot as plt
import control as ct

kt = 0.065 # N-m / A
R = 1.38 # Ohm
L = 2.26e-3 # H
kb = 0.065 # N-m / A
Bm = 1.21e-5 # N-m / rad /s
m1 = 0.277 # kg
m2 = 0.069 # kg
m3 = 0.009 # kg
L = 0.6096 # m
r = 0.0191 # m
r3 = 0.0127 # m 
I2 = 1/3 * m2 * L ** 2 + (2/5) * m3 * r3 ** 2 + m3 * L ** 2 
b = 0.1
Jm = 2.12e-5 # kg-m^2
sigma = 1 + Jm / (r ** 2 * (m1 + m2)) - (m2 * L) ** 2 / ((m1 + m2) * (I2 + m2 * L ** 2))
g = 9.81 # m/s^2

A = np.array(
    [
        [0, 1, 0, 0], 
        [0, -(Bm / (sigma * r ** 2 * (m1 + m2)) + (kt*kb)/(sigma * R * r ** 2 * (m1+m2))) + b/(sigma * (m1 + m2) ), -0.5 * (m2 * L) ** 2 * g / (sigma * (m1 + m2) * (I2 + m2 * L ** 2)), 0],
        [0, 0, 0, 1],
        [0, -( m2 * L * Bm / (sigma * r ** 2 * (m1 + m2) * (I2 + m2 * L ** 2)) + m2 * L * kt * kb / (sigma * R * r ** 2 * (m1 + m2) * (I2 + m2 * L ** 2)) + m2 * L * b / (sigma * (m1 + m2) * I2 + m2 * L ** 2) ), 0.5 * m2 * g * L * (I2 + m2 * L ** 2) - 0.5 * (m2 * L) ** 3 * g / (sigma * (m1 + m2) * (I2 + m2 * L ** 2) ** 2 ), 0]
    ]
)

B = np.array([
    0, 
    kt / (sigma * (m1 + m2) * R * r),
    0,
    -m2 * L * kt / (sigma * (m1 + m2) * (I2 + m2 * L  ** 2) * R * r)
]).reshape(-1, 1)

print(A)
print(B)

print("Controllable? ", np.linalg.matrix_rank(ct.ctrb(A, B)) == 4)

# Plot pole zero
C = np.eye(4)
D = np.zeros((4, 1))
sys = ct.ss(A, B, C, D)

# Print poles and zeros before pole placement  
"""
poles_before = ct.poles(sys)  
zeros_before = ct.zeros(sys)  
print("Poles before pole placement:", poles_before)  
print("Zeros before pole placement:", zeros_before)

# Plot Pole-Zero Map
plt.figure()
ct.pzmap(sys, title='Pole-Zero Map Before Pole Placement')
plt.show()

# Pole Placement
K = ct.place(A, B, [-1, -2, -3])
#K = ct.place(A, B, [-5-1j, -5+1j, -2])
print("K: ", K)

# New A matrix after pole placement
A_new = A - B @ K

# Create new state space system with new A matrix
sys_new = ct.ss(A_new, B, C, D)

# Plot Pole-Zero Map After Pole Placement
plt.figure()
ct.pzmap(sys_new, title='Pole-Zero Map After Pole Placement')
plt.show()
"""

# LQR
Q = np.array(
    [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
)

R = 1

K, S, E = ct.lqr(A, B, Q, R)
print("K: ", K)