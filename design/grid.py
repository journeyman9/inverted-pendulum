import numpy as np
import itertools
import control as ct
from model import A, B, C, D, sys, dsys

# ---------------------------------------------------------------------------
# Fixed Bryson denominators (acceptable error/effort ranges) — do not change.
# Only the numerators above each one are searched over.
# ---------------------------------------------------------------------------
DENOM = {
    "x": 0.1,
    "xdot": 0.651,
    "theta": 0.1,
    "thetadot": 0.5,
}
DENOM_R = 6.0

STATE_NAMES = ["x", "xdot", "theta", "thetadot"]


def build_QR(nx, nxdot, ntheta, nthetadot, nR):
    """Q[i,i] = numerator_i / denom_i**2, R = numerator_R / denom_R**2."""
    Q = np.diag([
        nx / DENOM["x"] ** 2,
        nxdot / DENOM["xdot"] ** 2,
        ntheta / DENOM["theta"] ** 2,
        nthetadot / DENOM["thetadot"] ** 2,
    ])
    R = nR / DENOM_R ** 2
    return Q, R


def closed_loop_modes(K):
    """Eigen-decompose A-BK and tag each mode with settling time + dominant state,
    using the same energy-based attribution as your existing script."""
    values, vectors = np.linalg.eig(A - B @ K)
    modes = []
    for i in range(len(values)):
        lam = values[i]
        v = vectors[:, i]
        sigma = np.real(lam)
        Ts = np.inf if sigma == 0 else 4 / np.abs(sigma)
        energy = np.abs(v) ** 2
        energy = energy / np.sum(energy)
        dominant = STATE_NAMES[np.argmax(energy)]
        modes.append({"eig": lam, "Ts": Ts, "dominant": dominant, "sigma": sigma})
    return modes


def thetadot_mode(modes):
    """Among modes most attributed to thetadot, take the slowest one — that's the
    one that actually governs how long thetadot takes to settle."""
    candidates = [m for m in modes if m["dominant"] == "thetadot"]
    if not candidates:
        return None
    return max(candidates, key=lambda m: m["Ts"])


def estimate_control_effort(K, test_states=None):
    """
    Estimate peak control voltage for typical disturbance scenarios.
    Control law: u = -K*x

    Tests several scenarios and returns max |u| in Volts.
    """
    if test_states is None:
        # Typical disturbance scenarios for inverted pendulum
        test_states = [
            [0.0, 0.0, 0.2, 0.0],      # 11.5° angle disturbance
            [0.0, 0.0, 0.15, 0.0],     # 8.6° angle disturbance
            [0.1, 0.0, 0.1, 0.0],      # combined position and angle
            [0.0, 0.5, 0.0, 0.5],      # velocity disturbances
            [0.1, 0.3, 0.15, 0.8],     # worst-case combination
        ]

    max_effort = 0.0
    for state in test_states:
        u = -K @ np.array(state).reshape(-1, 1)
        max_effort = max(max_effort, np.abs(u[0, 0]))

    return max_effort


def grid_search(grid_x, grid_xdot, grid_theta, grid_thetadot, grid_R,
                 max_settling_time=2.0, min_settling_time=0.15, max_voltage=24.0,
                 min_damping_ratio=0.3, verbose=True):
    """
    Sweep numerator values on a grid. For each combo:
      1. Build Q, R and solve LQR.
      2. Compute closed-loop metrics: settling time, damping, control effort (voltage).
      3. Filter by practical constraints (stability, voltage limits, damping, pole speed).
      4. Score by composite metric: fast settling, good damping, reasonable voltage.

    Returns (valid_results, all_results), both sorted best-score-first.
    valid_results only includes combos that satisfy all practical constraints.

    Control effort is estimated by applying K to typical disturbance states
    (e.g., theta=0.2 rad ≈ 11.5°) and checking peak voltage |u|.

    Args:
        min_settling_time: Minimum settling time (prevents poles that are too fast/aggressive).
                          Pole real part limited to: sigma < -4/min_settling_time.
                          Very fast poles (e.g., -25) cause practical instability.
    """
    combos = list(itertools.product(grid_x, grid_xdot, grid_theta, grid_thetadot, grid_R))
    if verbose:
        print(f"Searching {len(combos)} combinations...")

    results = []
    for nx, nxdot, ntheta, nthetadot, nR in combos:
        Q, R = build_QR(nx, nxdot, ntheta, nthetadot, nR)
        try:
            K, S, E = ct.lqr(sys, Q, R)
        except Exception:
            continue  # e.g. Riccati solve failure for a bad combo

        modes = closed_loop_modes(K)

        # Compute metrics
        max_Ts = max(m["Ts"] for m in modes if np.isfinite(m["Ts"]))
        min_Ts = min(m["Ts"] for m in modes if np.isfinite(m["Ts"]))
        all_stable = all(m["sigma"] < 0 for m in modes)

        # Check if any pole is too fast (causes practical instability)
        # Pole at sigma = -25 has Ts = 4/25 = 0.16s - too aggressive for real hardware
        fastest_pole = min(m["sigma"] for m in modes)  # most negative

        # Estimate peak voltage for typical disturbances
        peak_voltage = estimate_control_effort(K)

        # Compute minimum damping ratio for complex poles
        min_damping = 1.0  # assume all real poles are critically damped
        for m in modes:
            if np.abs(np.imag(m["eig"])) > 1e-6:  # complex pole
                omega_n = np.abs(m["eig"])
                zeta = -m["sigma"] / omega_n if omega_n > 0 else 0
                min_damping = min(min_damping, zeta)

        # Check if theta-dominant mode is stable and reasonably fast
        theta_modes = [m for m in modes if m["dominant"] in ["theta", "thetadot"]]
        theta_Ts = min((m["Ts"] for m in theta_modes), default=np.inf)

        # Composite score: prefer fast settling, good damping, reasonable voltage usage
        # Normalize each component to [0, 1] range
        settling_score = np.exp(-max_Ts / 1.0)  # prefer Ts < 1s
        damping_score = min(min_damping / 0.7, 1.0)  # prefer damping > 0.7
        voltage_score = np.exp(-peak_voltage / 6.0)  # prefer voltage < 6V

        # Penalize poles that are too fast (aggressive control, sensitive to delays/noise)
        # Prefer fastest pole around sigma ~ -10 to -20 range
        pole_speed_score = np.exp(-abs(fastest_pole + 15) / 10.0)  # prefer around -15

        # Weighted composite (tune these weights based on priorities)
        score = 0.3 * settling_score + 0.15 * damping_score + 0.4 * voltage_score + 0.15 * pole_speed_score

        # Practical validity checks
        valid = (all_stable and
                 max_Ts <= max_settling_time and
                 min_Ts >= min_settling_time and  # prevent overly aggressive poles
                 min_damping >= min_damping_ratio and
                 peak_voltage <= max_voltage)

        results.append({
            "nx": nx, "nxdot": nxdot, "ntheta": ntheta,
            "nthetadot": nthetadot, "nR": nR,
            "max_Ts": max_Ts,
            "min_Ts": min_Ts,
            "theta_Ts": theta_Ts,
            "fastest_pole": fastest_pole,
            "min_damping": min_damping,
            "peak_voltage": peak_voltage,
            "all_stable": all_stable,
            "valid": valid,
            "score": score,
            "K": K,
            "modes": modes,
        })

    results.sort(key=lambda r: -r["score"])  # higher score is better
    valid = [r for r in results if r["valid"]]
    return valid, results


def print_result(r, label="Result"):
    print(f"\n{label}:")
    print(f"  nx={r['nx']:.3g}  nxdot={r['nxdot']:.3g}  ntheta={r['ntheta']:.3g}  "
          f"nthetadot={r['nthetadot']:.3g}  nR={r['nR']:.3g}")
    print(f"  Settling time: {r['min_Ts']:.4f}s - {r['max_Ts']:.4f}s   "
          f"Fastest pole: {r['fastest_pole']:.2f}")
    print(f"  Min damping = {r['min_damping']:.3f}   Peak voltage = {r['peak_voltage']:.2f}V   "
          f"Score = {r['score']:.4f}")
    print(f"  K = {np.round(r['K'], 4)}")


if __name__ == "__main__":
    # Use logarithmic spacing for LQR weights (they span orders of magnitude)
    # Lower Q weights and higher R weights lead to less aggressive control (smaller gains, slower poles)
    grid_x = np.logspace(-1, 2, 8)        # 0.1 to 100
    grid_xdot = np.logspace(-1, 2, 6)     # 0.1 to 100
    grid_theta = np.logspace(-1, 2, 8)    # 0.1 to 100
    grid_thetadot = np.logspace(-1, 2, 6) # 0.1 to 100
    grid_R = np.logspace(1, 4, 8)         # 10 to 10000
    
    max_settling_time = 2.0
    min_settling_time = 0.2   # Prevent overly aggressive poles (e.g., -25 has Ts=0.16s)
    max_voltage = 6.0
    min_damping_ratio = 0.3

    valid, all_results = grid_search(
        grid_x, grid_xdot, grid_theta, grid_thetadot, grid_R,
        max_settling_time=max_settling_time,
        min_settling_time=min_settling_time,
        max_voltage=max_voltage,
        min_damping_ratio=min_damping_ratio,
    )

    print(f"\nTotal combinations evaluated: {len(all_results)}")

    if valid:
        print(f"\nFound {len(valid)} combo(s) meeting all constraints:")
        print("  - All poles stable")
        print("  - Settling time: {}s - {}s".format(min_settling_time, max_settling_time))
        print("  - Peak voltage ≤ {}V".format(max_voltage))
        print("  - Min damping ≥ {}".format(min_damping_ratio))
        print("\nTop 5 results by composite score:")
        for i in range(min(5, len(valid))):
            print_result(valid[i], label=f"#{i+1}")

    else:
        print("\nNo combo satisfied all constraints.")
        print("\nMost promising candidates (by score, may violate some constraints):")
        for i in range(min(3, len(all_results))):
            r = all_results[i]
            print_result(r, label=f"Candidate {i+1}")
            if not r['all_stable']:
                print("    ⚠ UNSTABLE")
            if r['peak_voltage'] > max_voltage:
                print(f"    ⚠ Voltage too high: {r['peak_voltage']:.1f}V > 24V")
            if r['max_Ts'] > max_settling_time:
                print(f"    ⚠ Too slow: Ts={r['max_Ts']:.2f}s > 2.0s")
            if r['min_damping'] < min_damping_ratio:
                print(f"    ⚠ Underdamped: ζ={r['min_damping']:.2f} < 0.3")

        # Give guidance for next iteration
        print("\n📊 Refinement suggestions:")
        print("  Adjust grid ranges around the most promising candidates above")
        print("  Or relax constraints (max_voltage, max_settling_time, min_damping_ratio)")