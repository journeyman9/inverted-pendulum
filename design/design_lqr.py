import numpy as np
import matplotlib.pyplot as plt
import control as ct
from model import A, B, C, D, sys, dsys
import pandas as pd

# LQR
Q = np.array(
    [
        [500, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 100, 0],
        [0, 0, 0, 0],
    ]
)

R = 1
'''
Q = np.array(
    [
        [177, 0, 0, 0],
        [0, 1.75, 0, 0],
        [0, 0, 1257, 0],
        [0, 0, 0, 0.53],
    ]
)

R = 0.25
'''
'''
Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 1 / (6 **2)
'''
'''
Q = np.array(
    [
        [1 / (0.2**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.0873 ** 2), 0],
        [0, 0, 0, 0],
    ]
)

R = 1 / (12 **2)
'''

## Now vary
'''
Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0.1 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 1 / (6 **2)

Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 10 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 1 / (6 **2)

Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 0.1 / (6 **2)

Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 10 / (6 **2)
'''

'''
# Brysons
Q = np.array(
    [
        [1/(0.1**2), 0, 0, 0],
        [0, 1/(0.1**2), 0, 0],
        [0, 0, 1/(0.05 ** 2), 0],
        [0, 0, 0, 1/(0.05 **2)],
    ]
)
R = 1 / (6 **2)
'''
'''
Q = np.array(
    [
        [1/(0.5**2), 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1/(0.1 ** 2), 0],
        [0, 0, 0, 1],
    ]
)
R = 0.1
'''
'''
Q = np.array(
    [
        [800, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 3500, 0],
        [0, 0, 0, 1],
    ]
)

R = 1
'''
'''
Q = np.array(
    [
        [10000, 0, 0, 0],
        [0, 1000, 0, 0],
        [0, 0, 0.1, 0],
        [0, 0, 0, 0.01],
    ]
)

R = 10
'''
'''
Q = np.array(
    [
        [23019, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 12500, 0],
        [0, 0, 0, 1],
    ]
)

R = 1
'''
'''
Q = np.array(
    [
        [500, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 400, 0],
        [0, 0, 0, 1],
    ]
)

R = 1
'''
'''
Q = np.array(
    [
        [386, 0, 0, 0],
        [0, 1.27, 0, 0],
        [0, 0, 128, 0],
        [0, 0, 0, 0.09],
    ]
)

R = 0.003
'''

K, S, E = ct.lqr(sys, Q, R)

print("CL: A-BK")
values, vectors = np.linalg.eig(A - B@K)
np.set_printoptions(precision=4, suppress=True)
for i in range(len(values)):
    print("x{} approx e ^ ({:.2f})t * {}".format(i, values[i], vectors[:, i]))

np.set_printoptions(precision=6, suppress=True)
K = K[0]
print("\nK: [{:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(K[0], K[1], K[2], K[3]))

# Evaluate actuator saturation
x0 = [0.2, 0, 0.0873, 0]
u = K @ x0
print("\nu: {:.6f}".format(u))

# Step 1: settling time per eigenvalue
Ts_modes = 4 / np.abs(np.real(values))

# Step 2: sort by dominance (slowest → fastest)
idx_sorted = np.argsort(np.abs(np.real(values)))  # small real part = dominant

def classify_mode(v):
    cart = np.abs(v[0]) + np.abs(v[1])
    pend = np.abs(v[2]) + np.abs(v[3])
    return "cart-dominant" if cart > pend else "pendulum-dominant"

rows = []

for i in idx_sorted:
    lam = values[i]
    v = vectors[:, i]

    sigma = np.real(lam)

    # settling time from eigenvalue (mode-based)
    Ts = np.inf if sigma == 0 else 4 / np.abs(sigma)

    rows.append({
        "Eigenvalue": lam,
        "Settling Time (s)": Ts,
        "Mode Type": classify_mode(v)
    })

mode_df = pd.DataFrame(rows)

mode_df["Eigenvalue (real)"] = mode_df["Eigenvalue"].apply(np.real)
mode_df["Eigenvalue (imag)"] = mode_df["Eigenvalue"].apply(np.imag)
mode_df = mode_df.drop(columns=["Eigenvalue"])

print("\n", mode_df.to_string(index=False, float_format=lambda x: f"{x:0.3f}"))

