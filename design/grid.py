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
DENOM_R = 2.0

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


def grid_search(grid_x, grid_xdot, grid_theta, grid_thetadot, grid_R,
                 target_Ts=0.8, tol=0.05, verbose=True):
    """
    Sweep numerator values on a grid. For each combo:
      1. Build Q, R and solve LQR.
      2. Find the thetadot-dominant mode's settling time.
      3. Require every other mode to settle faster than that mode.
      4. Score by |thetadot_Ts - target_Ts|.

    Returns (valid_results, all_results), both sorted best-match-first.
    valid_results only includes combos that satisfy BOTH the target tolerance
    AND the "other poles are faster" constraint.
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
        td_mode = thetadot_mode(modes)
        if td_mode is None:
            continue

        other_Ts = [m["Ts"] for m in modes if m is not td_mode]
        all_faster = all(ts < td_mode["Ts"] for ts in other_Ts)
        err = abs(td_mode["Ts"] - target_Ts)

        results.append({
            "nx": nx, "nxdot": nxdot, "ntheta": ntheta,
            "nthetadot": nthetadot, "nR": nR,
            "thetadot_Ts": td_mode["Ts"],
            "max_other_Ts": max(other_Ts) if other_Ts else np.nan,
            "all_faster": all_faster,
            "err": err,
            "K": K,
        })

    results.sort(key=lambda r: r["err"])
    valid = [r for r in results if r["all_faster"] and r["err"] <= tol]
    return valid, results


def print_result(r, label="Result"):
    print(f"\n{label}:")
    print(f"  nx={r['nx']:.3g}  nxdot={r['nxdot']:.3g}  ntheta={r['ntheta']:.3g}  "
          f"nthetadot={r['nthetadot']:.3g}  nR={r['nR']:.3g}")
    print(f"  thetadot Ts = {r['thetadot_Ts']:.4f}s   max other Ts = {r['max_other_Ts']:.4f}s   "
          f"all_faster={r['all_faster']}")
    print(f"  K = {np.round(r['K'], 4)}")


if __name__ == "__main__":
    # Coarse first pass — widen/narrow these ranges based on what you see.
    grid_x = np.linspace(0.5, 1000, 5)
    grid_xdot = np.linspace(0.5, 1000, 5)
    grid_theta = np.linspace(0.5, 1000, 6)
    grid_thetadot = np.linspace(0.1, 1000, 8)   # finest grid here — this is the one you're targeting
    grid_R = np.linspace(0.5, 1000, 6)

    valid, all_results = grid_search(
        grid_x, grid_xdot, grid_theta, grid_thetadot, grid_R,
        target_Ts=0.8, tol=0.05,
    )

    if valid:
        print(f"\nFound {len(valid)} combo(s) meeting target (Ts=0.8s ± 0.05) with all other poles faster.")
        #print_result(valid[0], label="Best match")
        for i in range(len(valid)):
            print_result(valid[i], label=i)
            
    else:
        print("\nNo combo satisfied both constraints within tolerance.")
        print("Closest by settling-time error (may violate the 'others faster' constraint):")
        print_result(all_results[0], label="Closest match")
        n_faster_ok = sum(1 for r in all_results if r["all_faster"])
        print(f"\n({n_faster_ok}/{len(all_results)} combos satisfied 'all other poles faster', "
              f"none hit the Ts tolerance — try narrowing grid_thetadot near the closest match above, "
              f"or relax tol.)")