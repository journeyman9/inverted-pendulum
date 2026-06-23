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

R = 75
''' Stable but not centering
Q = np.array(
    [
        [177, 0, 0, 0],
        [0, 1.75, 0, 0],
        [0, 0, 1257, 0],
        [0, 0, 0, 0.53],
    ]
)

R = 150
'''
''' Smooth but not centering
Q = np.array(
    [
        [1 / (0.3**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.05 ** 2), 0],
        [0, 0, 0, 0],
    ]
)
R = 100 / (6 **2)
'''
''' Smooth but not centering
Q = np.array(
    [
        [1 / (0.2**2), 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1 / (0.0873 ** 2), 0],
        [0, 0, 0, 0],
    ]
)

R = 100 / (12 ** 2)
'''

# Brysons
''' Did not work well
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
''' Something here
Q = np.array(
    [
        [750/(0.5**2), 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1/(0.1 ** 2), 0],
        [0, 0, 0, 1],
    ]
)
R = 50
'''
''' Stable not centering
Q = np.array(
    [
        [800, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 3500, 0],
        [0, 0, 0, 1],
    ]
)

R = 100
'''
''' Stable not centering
Q = np.array(
    [
        [10000, 0, 0, 0],
        [0, 1000, 0, 0],
        [0, 0, 0.1, 0],
        [0, 0, 0, 0.01],
    ]
)

R = 200
'''
''' Stable and wants to center, but not enough
Q = np.array(
    [
        [23019, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 12500, 0],
        [0, 0, 0, 1],
    ]
)

R = 300
'''
''' Stable but not centering
Q = np.array(
    [
        [500, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 400, 0],
        [0, 0, 0, 1],
    ]
)

R = 75
'''
''' Not centering enough
Q = np.array(
    [
        [386, 0, 0, 0],
        [0, 1.27, 0, 0],
        [0, 0, 128, 0],
        [0, 0, 0, 0.09],
    ]
)

R = 10
'''
''' Not centering enough
Q = np.array(
    [
        [100/(0.1**2), 0, 0, 0],
        [0, 0.001/(0.1**2), 0, 0],
        [0, 0, 1/(0.05**2), 0],
        [0, 0, 0, 0.001/(0.05**2)],
    ]
)
R = 20000 / (12 ** 2)
'''
''' Not centering enough
Q = np.array(
    [
        [100/(0.1**2), 0, 0, 0],
        [0, 0.001/(0.1**2), 0, 0],
        [0, 0, 0.05/(0.05**2), 0],
        [0, 0, 0, 0.001/(0.05**2)],
    ]
)
R = 20000 / (12 ** 2)
'''
''' Not centering enough
Q = np.array(
    [
        [10000/(0.1**2), 0, 0, 0],
        [0, 1/(0.1**2), 0, 0],
        [0, 0, 50/(0.05**2), 0],
        [0, 0, 0, 1/(0.05**2)],
    ]
)
R = 2000000 / (12 ** 2)
'''
''' Not centering enough
Q = np.array(
    [
        [10000/(0.05**2), 0, 0, 0],
        [0, 1/(0.1**2), 0, 0],
        [0, 0, 100/(0.0873**2), 0],
        [0, 0, 0, 1/(0.5**2)],
    ]
)
R = 10000000 / (12 ** 2)
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

idx_sorted = np.argsort(np.abs(np.real(values)))

state_names = ["x", "xdot", "theta", "thetadot"]

rows = []

for i in idx_sorted:
    lam = values[i]
    v = vectors[:, i]

    sigma = np.real(lam)
    Ts = np.inf if sigma == 0 else 4 / np.abs(sigma)

    # eigenvector energy contribution
    energy = np.abs(v)**2
    energy = energy / np.sum(energy)

    # NEW: dominant state index
    max_state_idx = np.argmax(energy)
    max_state = state_names[max_state_idx]

    rows.append({
        "Eigenvalue (real)": np.real(lam),
        "Eigenvalue (imag)": np.imag(lam),
        "Settling Time (s)": Ts,

        #"x contribution": energy[0],
        #"xdot contribution": energy[1],
        #"theta contribution": energy[2],
        #"thetadot contribution": energy[3],

        "Most Contributing State": max_state
    })

mode_df = pd.DataFrame(rows)

print("\n",
    mode_df.to_string(
        index=False,
        float_format=lambda x: f"{x:0.3f}"
    )
)

