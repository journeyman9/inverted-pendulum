import json
import os
import numpy as np
import matplotlib.pyplot as plt
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

C = np.eye(4)
D = np.zeros((4, 1))
sys = ct.ss(A, B, C, D)

# Discrete 
dt = 0.001
dsys = ct.c2d(sys, dt, method='zoh')

"""
wn = np.sqrt((m2 * g * L) / (I2 + m2*L**2))
print("Wn = {} rad/s".format(wn))

print("OL: A")
values, vectors = np.linalg.eig(A)
np.set_printoptions(precision=4, suppress=True)
for i in range(len(values)):
    print("x{} approx e ^ ({:.2f})t * {}".format(i, values[i], vectors[:, i]))

# Plot Pole-Zero Map
plt.figure()
ct.pzmap(sys, title='Pole-Zero Map Before Pole Placement')
plt.show()

# Pole Placement
# conservative
#poles = [-3+2j, -3-2j, -7, -9]

# more centered/faster
#poles = [-4+3j, -4-3j, -9, -12]

# aggressive
#poles = [-5+3j, -5-3j, -10, -14]

# Design from Wn and zeta
poles = [-7+7j, -7-7j, -15, -20]

K = ct.place(A, B, poles)

# New A matrix after pole placement
A_new = A - B @ K

# Create new state space system with new A matrix
sys_new = ct.ss(A_new, B, C, D)

eigvals, eigvecs = np.linalg.eig(A_new)

# Plot Pole-Zero Map After Pole Placement
plt.figure()
ct.pzmap(sys_new, title='Pole-Zero Map After Pole Placement')
plt.show()
"""

# LQR
'''
Q = np.array(
    [
        [10, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 100, 0],
        [0, 0, 0, 1],
    ]
)

R = 100
'''
'''
Q = np.array(
    [
        [10 / (0.4), 0, 0, 0],
        [0, 1 / (0.3), 0, 0],
        [0, 0, 100 / (0.384), 0],
        [0, 0, 0, 1 / (3.9)],
    ]
)

R = 100 / (8)
'''
'''
Q = np.array(
    [
        [1000, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 100, 0],
        [0, 0, 0, 1],
    ]
)

R = 40
'''
Q = np.array(
    [
        [10, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 10, 0],
        [0, 0, 0, 1],
    ]
)

R = 1

K, S, E = ct.lqr(sys, Q, R)

print("CL: A-BK")
values, vectors = np.linalg.eig(A - B@K)
np.set_printoptions(precision=4, suppress=True)
for i in range(len(values)):
    print("x{} approx e ^ ({:.2f})t * {}".format(i, values[i], vectors[:, i]))
 
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

np.set_printoptions(precision=6, suppress=True)
K = K[0]
print()
print("K: [{:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(K[0], K[1], K[2], K[3]))