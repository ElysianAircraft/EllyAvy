   # --- Built-ins ---
from pathlib import Path
import logging
import numbers

# --- Internal ---

# --- External ---
import numpy as np

logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

# -------------------------------------------------
# Treat None / empty containers as "unspecified"
# -------------------------------------------------
def _coerce_nan(val):
    """Return np.nan for None, [] or (), else leave untouched."""
    if val is None:
        return np.nan
    if isinstance(val, (list, tuple, np.ndarray)) and len(val) == 0:
        return np.nan
    return val


def power_transmission_computation(config, etas, supplied_power_ratio, shaft_power_ratio, throttle_setting, 
                                   P_p, P_p1, P_p2, P_inst)-> \
    tuple[float, float, float, float, float, dict]:
    """
    Translated from the MATLAB function 'PowerTransmissionComputationV3'.
    See the original MATLAB docstring for full details. This function
    computes the power flow along each power path for various hybrid
    powertrain configurations.

    Inputs:
    -------
    config : str
        Powertrain layout (e.g. 'conventional', 'turboelectric', 'serial', etc.)
    etas   : dict
        Dictionary of component efficiencies with fields:
        { 'GT', 'GB', 'P1', 'EM1', 'PM', 'EM2', 'P2' } (all in (0,1))
    supplied_power_ratio    : float or NaN
        Supplied power ratio [-],  P_bat / (P_bat + P_f)
    shaft_power_ratio    : float or NaN
        Shaft power ratio [-],  P_s2 / (P_s2 + P_s1)
    throttle_setting     : float or NaN
        Throttle setting [-], see notes in original docstring
    P_p    : float or NaN
        Total propulsive power [W]
    P_p1   : float or NaN
        Propulsive power from primary propulsor(s) [W]
    P_p2   : float or NaN
        Propulsive power from secondary propulsor(s) [W]
    P_inst : float or NaN
        Maximum installed power of GT or EM in this flight condition [W]

    Returns:
    --------
    P_out     : dict
        Dictionary with the power flow in each path
        { 'f','gt','gb','s1','e1','bat','e2','s2','p1','p2','p' }
    xi_out    : float
        Actual throttle used in the solution ([-])
    phi_out   : float
        Actual supplied power ratio ([-])
    Phi_out   : float
        Actual shaft power ratio ([-])
    solution  : np.ndarray of length 9
        A binary array indicating which of the 9 power-flow-direction
        combinations yielded a feasible solution
    xi_config : int
        Indicates which component throttle refers to:
        1 = Gas turbine, 2 = Primary EM, 3 = Secondary EM.
    """

    # --- 1) Assign input ---
    # --- 1) normalise "empty" inputs -------------------------------------
    supplied_power_ratio = _coerce_nan(supplied_power_ratio)
    shaft_power_ratio    = _coerce_nan(shaft_power_ratio)
    throttle_setting     = _coerce_nan(throttle_setting)
    P_p                  = _coerce_nan(P_p)
    P_p1                 = _coerce_nan(P_p1)
    P_p2                 = _coerce_nan(P_p2)
    P_inst               = _coerce_nan(P_inst)


    # Extract component efficiencies
    e_GT  = etas['GT']
    e_GB  = etas['GB']
    e_P1  = etas['P1']
    e_EM1 = etas['EM1']
    e_PM  = etas['PM']
    e_EM2 = etas['EM2']
    e_P2  = etas['P2']

    # Convert empty ([]) to NaN if needed
    # (Assuming you pass in np.nan already if not used. If not, you can add checks here.)

    # --- 2) Check input consistency with configuration ---

    # 2a) Ensure not all three propulsive powers are specified
    specified = [P_p, P_p1, P_p2]

    # --- Raise an error **only** when all three are supplied as real numbers
    if sum(specified) == 3:
        raise ValueError("Cannot specify P_p, P_p1, and P_p2 simultaneously.")

    # 2b) Ensure Pp, Pp1/Pp2, and Phi are not overspecified
    if (np.sum([~np.isnan(shaft_power_ratio), ~np.isnan(P_p1), ~np.isnan(P_p2)]) == 3 or
        np.sum([~np.isnan(shaft_power_ratio), ~np.isnan(P_p),  ~np.isnan(P_p1)]) == 3 or
        np.sum([~np.isnan(shaft_power_ratio), ~np.isnan(P_p),  ~np.isnan(P_p2)]) == 3):
        raise ValueError("Cannot specify Phi together with both P_p1 and P_p2 (or P_p).")

    # Initialize some variables we fill in the switch/case logic
    combis = []
    throttle_config = 1  # 1 = GT by default, see below

    config = config.lower()
    # Choose how many DOFs are expected, and fix some inputs depending on config
    if config == 'conventional':
        # Enforce OCs
        supplied_power_ratio = 0
        shaft_power_ratio = 0
        # The relevant set of input OCs
        OC = [throttle_setting, P_p, P_p1]
        DOFs = 1
        throttle_config = 1
        combis = [1]  # Which sign-combinations to check

    elif config == 'turboelectric':
        supplied_power_ratio = 0
        shaft_power_ratio = 1
        OC = [throttle_setting, P_p, P_p2]
        DOFs = 1
        throttle_config = 1
        combis = [1]

    elif config == 'serial':
        shaft_power_ratio = 1
        # possible OCs: phi, xi, P_p, P_p2
        OC = [supplied_power_ratio, throttle_setting, P_p, P_p2]
        DOFs = 2
        combis = [1, 2, 3]
        if supplied_power_ratio == 1:
            # Then throttle must refer to EM2
            throttle_config = 3

    elif config == 'parallel':
        shaft_power_ratio = 0
        # possible OCs: phi, xi, P_p, P_p1
        OC = [supplied_power_ratio, throttle_setting, P_p, P_p1]
        DOFs = 2
        combis = [1, 2, 4, 8]
        if supplied_power_ratio == 1:
            # Then throttle must refer to EM1
            throttle_config = 2

    elif config == 'PTE':
        supplied_power_ratio = 0
        # possible OCs: Phi, xi, P_p, P_p1, P_p2
        OC = [shaft_power_ratio, throttle_setting, P_p, P_p1, P_p2]
        DOFs = 2
        throttle_config = 1
        combis = [1, 5, 7]

    elif config == 'SPPH':
        # possible OCs: phi, Phi, xi, P_p, P_p1, P_p2
        OC = [supplied_power_ratio, shaft_power_ratio, throttle_setting, P_p, P_p1, P_p2]
        DOFs = 3
        combis = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        if supplied_power_ratio == 1:
            throttle_config = 3

    elif config == 'e-1':
        supplied_power_ratio = 1
        shaft_power_ratio = 0
        # possible OCs: xi, P_p, P_p1
        OC = [throttle_setting, P_p, P_p1]
        DOFs = 1
        throttle_config = 2
        combis = [4, 8]

    elif config == 'e-2':
        supplied_power_ratio = 1
        shaft_power_ratio = 1
        # possible OCs: xi, P_p, P_p2
        OC = [throttle_setting, P_p, P_p2]
        DOFs = 1
        throttle_config = 3
        combis = [1, 3]

    elif config == 'dual-e':
        supplied_power_ratio = 1
        # possible OCs: xi, Phi, P_p, P_p1, P_p2
        OC = [throttle_setting, shaft_power_ratio, P_p, P_p1, P_p2]
        DOFs = 2
        throttle_config = 2
        combis = [4, 5, 6, 7, 8, 9]

    else:
        raise ValueError("Unrecognized config option: " + str(config))

    # Check number of DOFs actually specified vs required
    # "OC" here is just a subset of possible inputs - count how many are not NaN
    OC = [_coerce_nan(iOC) for iOC in OC]
    if np.sum(~np.isnan(OC)) != DOFs:
        print(OC, DOFs)
        raise ValueError(
            f"For a '{config}' configuration, exactly {DOFs} DOF(s) must be specified. "
            "Check your inputs."
        )

    # If throttle is given, P_inst must also be given
    if (not np.isnan(throttle_setting)) and np.isnan(P_inst):
        raise ValueError(
            "If throttle xi is specified, you must supply P_inst for the GT or EM."
        )

    # --- 3) Zero-power check: if the total powers & throttle are all zero, solution is trivial ---
    # Identify if none of P_p, P_p1, P_p2, or xi*P_inst are nonzero
    def is_zero_or_nan(x):
        if x is None:
            return True
        else:
            return (np.isnan(x) or x == 0.0)

    P_pSwitch  = 0 if is_zero_or_nan(P_p)  else 1
    P_p1Switch = 0 if is_zero_or_nan(P_p1) else 1
    P_p2Switch = 0 if is_zero_or_nan(P_p2) else 1
    if (not np.isnan(throttle_setting)) and (not np.isnan(P_inst)):
        xiSwitch = 0 if (throttle_setting == 0 or P_inst == 0) else 1
    else:
        xiSwitch = 0

    zeroIsASolution = 0
    if (P_pSwitch + P_p1Switch + P_p2Switch + xiSwitch) == 0:
        zeroIsASolution = 1

    # --- 4) Build system matrix A_top for the 9 flow‐direction cases (7 eqs per combination) ---
    # A_top has shape (7, 10, 9). We'll fill them with np.nan and then plug in each row.
    A_top = np.full((7, 10, 9), np.nan)

    # For code clarity, define indices for each power path column:
    # 0=Pf, 1=Pgt, 2=Pgb, 3=Ps1, 4=Pe1, 5=Pbat, 6=Pe2, 7=Ps2, 8=Pp1, 9=Pp2

    # -- CASE 1: nominal (all positive)
    A_top[:,:,0] = np.array([
        [-e_GT,  1,      0,      0,      0,      0,      0,      0,      0,      0],
        [0,     -e_GB,   1,      1,      0,      0,      0,      0,      0,      0],
        [0,      0,      0,     -e_P1,   0,      0,      0,      0,      1,      0],
        [0,      0,     -e_EM1,  0,      1,      0,      0,      0,      0,      0],
        [0,      0,      0,      0,     -e_PM,  -e_PM,   1,      0,      0,      0],
        [0,      0,      0,      0,      0,      0,     -e_EM2,  1,      0,      0],
        [0,      0,      0,      0,      0,      0,      0,     -e_P2,   0,      1]
    ])

    # -- CASE 2
    A_top[:,:,1] = np.array([
        [-e_GT,  1,      0,      0,      0,      0,      0,      0,      0,      0],
        [0,     -e_GB,   1,      1,      0,      0,      0,      0,      0,      0],
        [0,      0,      0,     -e_P1,   0,      0,      0,      0,      1,      0],
        [0,      0,     -e_EM1,  0,      1,      0,      0,      0,      0,      0],
        [0,      0,      0,      0,     -e_PM,  -1,      1,      0,      0,      0],
        [0,      0,      0,      0,      0,      0,     -e_EM2,  1,      0,      0],
        [0,      0,      0,      0,      0,      0,      0,     -e_P2,   0,      1]
    ])

    # -- CASE 3
    A_top[:,:,2] = np.array([
        [-e_GT,   1,       0,       0,      0,      0,       0,       0,      0,      0],
        [0,      -e_GB,    1,       1,      0,      0,       0,       0,      0,      0],
        [0,       0,       0,      -e_P1,   0,      0,       0,       0,      1,      0],
        [0,       0,      -e_EM1,   0,      1,      0,       0,       0,      0,      0],
        [0,       0,       0,       0,     -e_PM,  -1,       e_PM,    0,      0,      0],
        [0,       0,       0,       0,      0,      0,      -1,       e_EM2,  0,      0],
        [0,       0,       0,       0,      0,      0,       0,      -1,      0,      e_P2]
    ])

    # -- CASE 4
    A_top[:,:,3] = np.array([
        [-e_GT,  1,       0,       0,       0,      0,      0,      0,      0,      0],
        [0,     -e_GB,    e_GB,    1,       0,      0,      0,      0,      0,      0],
        [0,      0,       0,      -e_P1,    0,      0,      0,      0,      1,      0],
        [0,      0,      -1,       0,       e_EM1,  0,      0,      0,      0,      0],
        [0,      0,       0,       0,      -1,     -e_PM,   1,      0,      0,      0],
        [0,      0,       0,       0,       0,      0,     -e_EM2,  1,      0,      0],
        [0,      0,       0,       0,       0,      0,      0,     -e_P2,   0,      1]
    ])

    # -- CASE 5
    A_top[:,:,4] = np.array([
        [-e_GT,  1,       0,       0,      0,      0,      0,       0,      0,      0],
        [0,     -e_GB,    e_GB,    1,      0,      0,      0,       0,      0,      0],
        [0,      0,       0,      -e_P1,   0,      0,      0,       0,      1,      0],
        [0,      0,      -1,       0,      e_EM1,  0,      0,       0,      0,      0],
        [0,      0,       0,       0,     -1,     -e_PM,   e_PM,    0,      0,      0],
        [0,      0,       0,       0,      0,      0,     -1,       e_EM2,  0,      0],
        [0,      0,       0,       0,      0,      0,      0,      -1,      0,      e_P2]
    ])

    # -- CASE 6
    A_top[:,:,5] = np.array([
        [-e_GT,  1,       0,       0,      0,       0,       0,       0,      0,      0],
        [0,     -e_GB,    e_GB,    1,      0,       0,       0,       0,      0,      0],
        [0,      0,       0,      -e_P1,   0,       0,       0,       0,      1,      0],
        [0,      0,      -1,       0,      e_EM1,   0,       0,       0,      0,      0],
        [0,      0,       0,       0,     -1,      -1,       e_PM,    0,      0,      0],
        [0,      0,       0,       0,      0,       0,      -1,       e_EM2,  0,      0],
        [0,      0,       0,       0,      0,       0,       0,      -1,      0,      e_P2]
    ])

    # -- CASE 7
    A_top[:,:,6] = np.array([
        [-e_GT,  1,      0,       0,      0,      0,      0,      0,      0,      0],
        [0,     -e_GB,   1,       e_GB,   0,      0,      0,      0,      0,      0],
        [0,      0,      0,      -1,      0,      0,      0,      0,      e_P1,   0],
        [0,      0,     -e_EM1,   0,      1,      0,      0,      0,      0,      0],
        [0,      0,      0,       0,     -e_PM,  -e_PM,   1,      0,      0,      0],
        [0,      0,      0,       0,      0,      0,     -e_EM2,  1,      0,      0],
        [0,      0,      0,       0,      0,      0,      0,     -e_P2,   0,      1]
    ])

    # -- CASE 8
    A_top[:,:,7] = np.array([
        [-e_GT,  1,      0,       0,      0,      0,      0,      0,      0,      0],
        [0,     -e_GB,   1,       e_GB,   0,      0,      0,      0,      0,      0],
        [0,      0,      0,      -1,      0,      0,      0,      0,      e_P1,   0],
        [0,      0,     -e_EM1,   0,      1,      0,      0,      0,      0,      0],
        [0,      0,      0,       0,     -e_PM,  -1,      1,      0,      0,      0],
        [0,      0,      0,       0,      0,      0,     -e_EM2,  1,      0,      0],
        [0,      0,      0,       0,      0,      0,      0,     -e_P2,   0,      1]
    ])

    # -- CASE 9
    A_top[:,:,8] = np.array([
        [-e_GT,  1,      0,       0,      0,      0,      0,      0,      0,      0],
        [0,     -e_GB,   1,       e_GB,   0,      0,      0,      0,      0,      0],
        [0,      0,      0,      -1,      0,      0,      0,      0,      e_P1,   0],
        [0,      0,     -e_EM1,   0,      1,      0,      0,      0,      0,      0],
        [0,      0,      0,       0,     -e_PM,  -1,      e_PM,   0,      0,      0],
        [0,      0,      0,       0,      0,      0,     -1,      e_EM2,  0,      0],
        [0,      0,      0,       0,      0,      0,      0,     -1,      0,      e_P2]
    ])

    # b_top is always zeros for the top 7 eqs
    b_top = np.zeros(7)

    # --- 5) Select & build the 3 "operating condition" equations (A_bot, b_bot) ---
    A_bot = np.full((3,10), np.nan)
    b_bot = np.full(3, np.nan)

    # We'll fill row by row:
    eqN = 0

    # Helper function to store row in A_bot, b_bot:
    def set_eq(row_vec, rhs):
        nonlocal eqN
        A_bot[eqN,:] = row_vec
        b_bot[eqN]   = rhs
        eqN += 1

    # The order of "k" below matters because we match the original code's
    # indexing for phi (k=1), Phi (k=2), xi (k=3), P_p (k=4), P_p1 (k=5), P_p2 (k=6).
    # We'll just manually check each input in that order:
    if not np.isnan(supplied_power_ratio):  # k=1
        # A_bot(eqN,:) = [phi, 0, 0, 0, 0, (phi-1), 0, 0, 0, 0]
        set_eq([ supplied_power_ratio, 0,   0,   0,   0, (supplied_power_ratio - 1), 0,   0,   0,  0 ], 0.0)

    if not np.isnan(shaft_power_ratio):  # k=2
        # A_bot(eqN,:) = [0, 0, 0, Phi, 0, 0, 0, (Phi-1), 0, 0]
        set_eq([ 0,   0,  0,  shaft_power_ratio,  0,   0,   0, (shaft_power_ratio-1), 0,  0 ], 0.0)

    if not np.isnan(throttle_setting):   # k=3
        # depends on xi_config
        if throttle_config == 1:   # GT throttle
            row = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
        elif throttle_config == 2: # e-1 or dual-e => e1 path
            # Note we have negative e1 in the nominal direction for e-1
            row = [0, 0, 0, 0, -1, 0, 0, 0, 0, 0]
        else:                # xi_config == 3 => e2 path
            row = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]

        set_eq(row, throttle_setting * P_inst)

    if not np.isnan(P_p):  # k=4
        # A_bot(eqN,:) = [0,0,0,0,0,0,0,0,1,1]
        set_eq([0, 0, 0, 0, 0, 0, 0, 0, 1, 1], P_p)

    if not np.isnan(P_p1): # k=5
        set_eq([0, 0, 0, 0, 0, 0, 0, 0, 1, 0], P_p1)

    if not np.isnan(P_p2): # k=6
        set_eq([0, 0, 0, 0, 0, 0, 0, 0, 0, 1], P_p2)

    # If fewer than 3 eqs were set, fill the remainder with zeros:
    while eqN < 3:
        A_bot[eqN,:] = 0
        b_bot[eqN]   = 0
        eqN += 1

    # Merge top/bot to form full system: A and b
    # top (7 eqs) + bot (3 eqs) => 10 eqs total, each eq is length 10
    # shape: A = (10,10), b = (10,)
    b = np.concatenate([b_top, b_bot])

    # --- 6) Solve for each of the possible 9 sign-combinations, check feasibility ---
    P_sol = np.full((9,10), np.nan)  # Each row is a solution for that combi
    solution = np.zeros(9, dtype=int)

    for combi in combis:
        # combi is 1..9 in MATLAB, but 0‐indexed in Python we do combi-1?
        # The code above used 'case k' in [1..9]. We'll do k-1 in Python:
        k_idx = combi - 1
        A_mat = np.vstack([A_top[:,:,k_idx], A_bot])
        try:
            # Solve linear system
            x_vec = np.linalg.solve(A_mat, b)
        except np.linalg.LinAlgError:
            # If singular or ill-conditioned, skip
            continue

        P_sol[k_idx,:] = x_vec

        # Check feasibility w.r.t. sign assumptions.
        # Compare to MATLAB cases. We check columns:
        #   Pgt(1), Ps1(3), Pe1(4), Pbat(5), Pe2(6) with correct sign.
        # Indices are [1, 3, 4, 5, 6] in 0-based => [1, 3, 4, 5, 6]
        # For each combi, see original sign logic:
        pgt = x_vec[1]
        ps1 = x_vec[3]
        pe1 = x_vec[4]
        pbat = x_vec[5]
        pe2 = x_vec[6]

        # We do it exactly as the original switch/case:
        if combi == 1:
            if (pgt >= 0 and ps1 >= 0 and pe1 >= 0 and pbat >= 0 and pe2 >= 0):
                solution[k_idx] = 1
        elif combi == 2:
            if (pgt >= 0 and ps1 >= 0 and pe1 >= 0 and pbat < 0 and pe2 >= 0):
                solution[k_idx] = 1
        elif combi == 3:
            if (pgt >= 0 and ps1 >= 0 and pe1 >= 0 and pbat < 0 and pe2 < 0):
                solution[k_idx] = 1
        elif combi == 4:
            if (pgt >= 0 and ps1 >= 0 and pe1 < 0 and pbat >= 0 and pe2 >= 0):
                solution[k_idx] = 1
        elif combi == 5:
            if (pgt >= 0 and ps1 >= 0 and pe1 < 0 and pbat >= 0 and pe2 < 0):
                solution[k_idx] = 1
        elif combi == 6:
            if (pgt >= 0 and ps1 >= 0 and pe1 < 0 and pbat < 0 and pe2 < 0):
                solution[k_idx] = 1
        elif combi == 7:
            if (pgt >= 0 and ps1 < 0 and pe1 >= 0 and pbat >= 0 and pe2 >= 0):
                solution[k_idx] = 1
        elif combi == 8:
            if (pgt >= 0 and ps1 < 0 and pe1 >= 0 and pbat < 0 and pe2 >= 0):
                solution[k_idx] = 1
        elif combi == 9:
            if (pgt >= 0 and ps1 < 0 and pe1 >= 0 and pbat < 0 and pe2 < 0):
                solution[k_idx] = 1

    # --- 7) Final check for multiple solutions or none ---
    n_solutions = np.sum(solution)
    if n_solutions > 1:
        # Typically pick the first solution that isn't all zeros
        feasible_indices = np.where(solution == 1)[0]
        # if the first one is not the all-zero solution, pick it
        solIdx = feasible_indices[0]
        # Optionally check if it is truly different from the others, etc.
        if not np.allclose(P_sol[solIdx,:], 0.0):
            print(f"Warning: multiple solutions found ({n_solutions}). "
                  f"Selecting the first feasible (combis={solution.tolist()}).")
    elif n_solutions == 0:
        if zeroIsASolution == 1:
            # All powers are zero if everything is zero or NaN => choose index= -1
            solIdx = -1
        else:
            print("Warning: no solutions found.")
            solIdx = None
    else:
        # exactly one solution
        solIdx = np.argmax(solution)  # index of first (and only) '1'

    # --- 8) Organize outputs ---

    if solIdx is None:
        # no solution
        P_out = {name: np.nan for name in 
                 ['f','gt','gb','s1','e1','bat','e2','s2','p1','p2','p']}
    elif solIdx == -1:
        # zero solution
        P_out = {name: 0.0 for name in
                 ['f','gt','gb','s1','e1','bat','e2','s2','p1','p2']}
        P_out['p'] = 0.0
    else:
        # Retrieve the feasible solution
        x_vec = P_sol[solIdx, :]
        P_out = {
            'f'   : x_vec[0],
            'gt'  : x_vec[1],
            'gb'  : x_vec[2],
            's1'  : x_vec[3],
            'e1'  : x_vec[4],
            'bat' : x_vec[5],
            'e2'  : x_vec[6],
            's2'  : x_vec[7],
            'p1'  : x_vec[8],
            'p2'  : x_vec[9],
        }
        P_out['p'] = P_out['p1'] + P_out['p2']

        # Null-out unused components depending on config
        if config == 'conventional':
            P_out['gb']  = np.nan
            P_out['e1']  = np.nan
            P_out['bat'] = np.nan
            P_out['e2']  = np.nan
            P_out['s2']  = np.nan
            P_out['p2']  = np.nan

        elif config == 'turboelectric':
            P_out['s1']  = np.nan
            P_out['p1']  = np.nan
            P_out['bat'] = np.nan

        elif config == 'serial':
            P_out['s1']  = np.nan
            P_out['p1']  = np.nan

        elif config == 'parallel':
            P_out['s2']  = np.nan
            P_out['p2']  = np.nan
            P_out['e2']  = np.nan

        elif config == 'PTE':
            P_out['bat'] = np.nan

        elif config == 'SPPH':
            pass  # uses everything

        elif config == 'e-1':
            P_out['f']   = np.nan
            P_out['gt']  = np.nan
            P_out['e2']  = np.nan
            P_out['s2']  = np.nan
            P_out['p2']  = np.nan

        elif config == 'e-2':
            P_out['f']   = np.nan
            P_out['gt']  = np.nan
            P_out['gb']  = np.nan
            P_out['s1']  = np.nan
            P_out['p1']  = np.nan
            P_out['e1']  = np.nan

        elif config == 'dual-e':
            P_out['f']   = np.nan
            P_out['gt']  = np.nan

    # Compute the final xi_out, phi_out, Phi_out
    if not np.isnan(P_inst):
        if throttle_config == 1:
            xi_out = (P_out['gt'] / P_inst) if not np.isnan(P_out['gt']) else np.nan
        elif throttle_config == 2:
            # e1 is negative in normal usage as generator, so throttle = -P_e1 / P_inst
            xi_out = (-P_out['e1'] / P_inst) if not np.isnan(P_out['e1']) else np.nan
        else:
            xi_out = (P_out['e2'] / P_inst) if not np.isnan(P_out['e2']) else np.nan
    else:
        xi_out = np.nan

    if np.isnan(supplied_power_ratio):
        # P_out['bat'] / (P_out['bat'] + P_out['f'])
        den = (P_out['bat'] + P_out['f'])
        phi_out = (P_out['bat'] / den) if abs(den) > 1e-12 else np.nan
    else:
        phi_out = supplied_power_ratio

    if np.isnan(shaft_power_ratio):
        den = (P_out['s2'] + P_out['s1'])
        Phi_out = (P_out['s2'] / den) if abs(den) > 1e-12 else np.nan
    else:
        Phi_out = shaft_power_ratio

    return P_out, xi_out, phi_out, Phi_out, solution, throttle_config
