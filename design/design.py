import json
import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct

R = 1.38 # Ohm
kt = 0.065 # N-m / A
kb = 0.065 # N-m / A
Bm = 1.21e-5 # N-m / rad /s
m1 = 0.277 # kg
ms = 0.009 # kg
mr = 0.069
m2 = mr + ms # kg
l = 0.6096 # m
r = 0.0191 # m
r3 = 0.0127 # m 
L = (1 / (mr + ms)) * (mr * (l/2) + ms * l)

I2 = 1/3 * mr * l ** 2 + (2/5) * ms * r3 ** 2 + ms * l ** 2 - (m2 * L ** 2) # Subtracting to not double count
b = 0.1
Jm = 2.12e-5 # kg-m^2
sigma = 1 + (Jm / (r ** 2 * (m1 + m2))) - ((m2 * L) ** 2 / ((m1 + m2) * (I2 + m2 * L ** 2)))
g = 9.81 # m/s^2

A = np.array(
    [
        [0, 1, 0, 0], 
        [0, (-1.0 / (sigma * (m1 + m2))) * ( Bm / (r ** 2 ) + (kt*kb)/(R * r ** 2) + b), (1.0 / (sigma * (m1 + m2))) * (m2 * L) ** 2 * g / (I2 + m2 * L ** 2), 0],
        [0, 0, 0, 1],
        [0, (-1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * ( m2 * L) * ( Bm / (r ** 2) + (kt * kb) / (R * r ** 2) + b), (1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * (m2 * L * g) * (m1 + m2 + (Jm / (r ** 2))), 0]
    ]
)

B = np.array([
    0, 
    (1.0 / (sigma * (m1 + m2))) * kt / (R * r),
    0,
    (1.0 / (sigma * (m1 + m2) * (I2 + m2 * L ** 2))) * (m2 * L) * ( kt / (R * r) )
]).reshape(-1, 1)

# print(A)
# print(B)

# print("Controllable? ", np.linalg.matrix_rank(ct.ctrb(A, B)) == 4)

# Plot pole zero
C = np.eye(4)
D = np.zeros((4, 1))
sys = ct.ss(A, B, C, D)

# print("Eigenvalues OL")
values, vectors = np.linalg.eig(A)
# print(values)

# Discrete 
dt = 0.001
dsys = ct.c2d(sys, dt, method='zoh')

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
        [10, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 100, 0],
        [0, 0, 0, 1],
    ]
)

R = 0.01

"""
Q = np.array(
    [
        [1 / (0.4 ** 2), 0, 0, 0],
        [0, 1 / (0.3 ** 2), 0, 0],
        [0, 0, 1 / (0.3925 ** 2), 0],
        [0, 0, 0, 1 / (3.9 ** 2)],
    ]
)

R = 1 / (24 ** 2)
"""

K, S, E = ct.dlqr(dsys, Q, R)

db_path = os.path.join(os.path.dirname(__file__), "k_q_values.json")
if os.path.exists(db_path):
    with open(db_path, "r") as f:
        db = json.load(f)
else:
    db = {}

q_key = str(Q.tolist())
existing_notes = db.get(q_key, {}).get("notes", "")
db[q_key] = {"Q": Q.tolist(), "R": R if np.isscalar(R) else np.array(R).tolist(), "K": K.tolist(), "notes": existing_notes}

def _is_numeric_list(obj):
    if not isinstance(obj, list) or not obj:
        return False
    return (all(isinstance(x, (int, float)) for x in obj) or
            all(isinstance(x, list) and all(isinstance(y, (int, float)) for y in x) for x in obj))

def _json_compact(obj, indent=0):
    pad, inner = ' ' * indent, ' ' * (indent + 2)
    if isinstance(obj, dict):
        parts = []
        keys = list(obj.keys())
        for i, k in enumerate(keys):
            comma = ',' if i < len(keys) - 1 else ''
            parts.append(f'{inner}{json.dumps(k)}: {_json_compact(obj[k], indent + 2)}{comma}')
        return '{\n' + '\n'.join(parts) + '\n' + pad + '}'
    elif _is_numeric_list(obj):
        return json.dumps(obj)
    else:
        return json.dumps(obj)

with open(db_path, "w") as f:
    f.write(_json_compact(db))

print("K: ", K)