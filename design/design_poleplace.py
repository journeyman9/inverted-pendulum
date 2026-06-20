import numpy as np
import matplotlib.pyplot as plt
import control as ct
from model import A, B, C, D, wn_pend, sys, dsys

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
#poles = [-5+2j, -5-2j, -10, -11] # Best right now
#poles = [-7, -24.92, -6.36, -6.51] # Match equally fast unstable pole, move integrator faster
#poles = [-3+12j, -3-12j, -2.4+2j, -2.4-2j] # Should it have two conjugate poles?

# Design from Wn and zeta
#poles = [-5.208+1.711j, -5.208-1.711j, -1, -30] # zeta=0.95, wn=5.482 (better centering, needs help balancing)
#poles = [-4.557+1.498j, -4.557-1.498j, -1, -20] # zeta=0.95, wn=4.797 try this
#poles = [-7.812+2.568j, -7.812-2.568j, -1, -5] # zeta=0.95 , wn=8.223 (terrible, not good at either task)
#poles = [-7.812+10.416j, -7.812-10.416j, -1, -2] # zeta=0.6, wn=13.02 (sluggish)
#poles = [-5.208+6.944j, -5.208-6.944j, -1, -2] # zeta=0.6, wn=8.68 (terrible, hardly moves)
#poles = [-5+3.75j, -5-3.75j, -1, -20] # zeta=0.8, wn=6.25 (stays upright, but doesnt center)
#poles = [-2+3.9040j, -2-3.9040j, -5, -10] # zeta=0.4559 20% OS, wn=4.38664 settling time 2s

settling_time_target = 2.0 # 4.0
overshoot_target = 20.0 # 10.0

# Dominant pole real
dominant_real = -4.0 / settling_time_target

PO = overshoot_target / 100.0
if PO <= 0:
    zeta = 1.0
else:
    zeta = -np.log(PO) / np.sqrt(np.pi**2 + (np.log(PO))**2)

wn = abs(dominant_real) / zeta
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
