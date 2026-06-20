import numpy as np
import control as ct

R = 1.38 # Ohm
kt = 0.065 # N-m / A
kb = 0.065 # N-m / A
Bm = 1.21e-5 # N-m / rad /s
m1 = 0.280 # kg
ms = 0.008 # kg
mr = 0.035 # kg
m2 = mr + ms # kg
l = 0.3048 # m
r = 0.0188 # m
r3 = 0.0181 # m 
L = (1 / (mr + ms)) * (mr * (l/2) + ms * l) # COM

Ipivot = 1/3 * mr * l ** 2 + (2/5) * ms * r3 ** 2 + ms * l ** 2
I2 = Ipivot - (m2 * L ** 2) # Subtracting to not double count
b = 1.0
m_pulley = 0.035 # kg
J_pulley = 0.5 * m_pulley * r ** 2
m_idler = 0.048 #
J_idler = 0.5 * m_idler * r ** 2
Jm = 2.12e-5  # kg-m^2
Jt = Jm + J_pulley + J_idler
sigma = 1 + (Jt / (r ** 2 * (m1 + m2))) - ((m2 * L) ** 2 / ((m1 + m2) * (I2 + m2 * L ** 2)))
g = 9.81 # m/s^2

A = np.array(
    [
        [0, 1, 0, 0], 
        [0, (-1.0 / (sigma * (m1 + m2))) * ( Bm / (r ** 2 ) + (kt*kb)/(R * r ** 2) + b), (1.0 / (sigma * (m1 + m2))) * (m2 * L) ** 2 * g / (I2 + m2 * L ** 2), 0],
        [0, 0, 0, 1],
        [0, (-1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * ( m2 * L) * ( Bm / (r ** 2) + (kt * kb) / (R * r ** 2) + b), (1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * (m2 * L * g) * (m1 + m2 + (Jt / (r ** 2))), 0] ]
)

B = np.array([
    0, 
    (1.0 / (sigma * (m1 + m2))) * kt / (R * r),
    0,
    (1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * (m2 * L) * ( kt / (R * r) )
]).reshape(-1, 1)

#print(A)
#print(B)

# print("Controllable? ", np.linalg.matrix_rank(ct.ctrb(A, B)) == 4)

# Natural frequency of pendulum on its own
wn_pend = np.sqrt((m2 * g * L) / (I2 + m2*L**2))

C = np.eye(4)
D = np.zeros((4, 1))
sys = ct.ss(A, B, C, D)

# Discrete 
dt = 0.001
dsys = ct.c2d(sys, dt, method='zoh')