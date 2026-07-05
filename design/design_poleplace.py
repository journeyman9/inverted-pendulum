import numpy as np
import matplotlib.pyplot as plt
import control as ct
from model import A, B, C, D, wn_pend, sys, dsys
import pandas as pd

print("Wn = {} rad/s".format(wn_pend))

print("\nOL: A")
values, vectors = np.linalg.eig(A)
np.set_printoptions(precision=4, suppress=True)
for i in range(len(values)):
    print("x{} approx e ^ ({:.2f})t * {}".format(i, values[i], vectors[:, i]))

# Plot Pole-Zero Map
#plt.figure()
#ct.pzmap(sys, title='Pole-Zero Map Before Pole Placement')
#plt.show()

# Pole Placement
#poles = [-5+3j, -5-3j, -10, -14] # Doing it, but occasionally unstable
#poles = [-5+2j, -5-2j, -10, -11] # First gains that worked
#poles = [-4.55+4.55j, -4.55-4.55j, -9.65+7.75j, -9.65-7.75j]

# Design from Wn and zeta
# poles = [] settling time 0.7, overshoot target 0.001 # Doing better 2/3 mellow
# poles = [] settling time 0.8, overshoot target 1 # Doing better 1/3 centering more
# poles = [] settling time 0.9, overshoot target 4.33 # doing better 3/3
# poles = [] settling time 0.88, overshoot target 4.33 # Doesnt work

settling_time_target = 0.8
overshoot_target = 1.0

# Dominant pole real
dominant_real = -4.0 / settling_time_target

PO = overshoot_target / 100.0
if PO <= 0:
    zeta = 1.0
else:
    zeta = -np.log(PO) / np.sqrt(np.pi**2 + (np.log(PO))**2)
    
print("\nzeta: {:.3f}".format(zeta))

wn = abs(dominant_real) / zeta
print("\nWn: {:.3f}".format(wn))

dominant_pair = [complex(dominant_real, wn * np.sqrt(1 - zeta**2)),
                 complex(dominant_real, -wn * np.sqrt(1 - zeta**2))]

# Place remaining poles further left 
remaining_poles = [-wn*1.5, -wn*2.0] # or 5x dominant pole
poles = np.array(dominant_pair + remaining_poles)

K = ct.place(A, B, poles)

# Plot Pole-Zero Map After Pole Placement
#plt.figure()
#ct.pzmap(sys_new, title='Pole-Zero Map After Pole Placement')
#plt.show()

print("\nCL: A-BK")
values, vectors = np.linalg.eig(A - B @ K)
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