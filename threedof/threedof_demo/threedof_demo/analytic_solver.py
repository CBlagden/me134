import numpy as np

L1 = 0.508
L2 = 0.316

x_off, y_off, z_off = (
    0.04970353424430905,
    0.050472416460127276,
    0.12211428352936989,
)

def get_sols(x, y, z):
    # offset goal position to be described in frame of middle joint
    x -= x_off
    y -= y_off
    z -= z_off

    R = np.sqrt(x * x + y * y)
    THETA2 = np.arccos((x * x + y * y + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2))

    def pSol(r, theta2):
        thetapan = np.arctan2(x / r, -y / r)
        theta1 = np.arctan2(z, r) - np.arctan2(
            L1 + L2 * np.cos(theta2), L2 * np.sin(theta2)
        )
        return [thetapan, theta1, theta2]

    pA = pSol(R, THETA2)
    pB = pSol(-R, THETA2)
    pC = pSol(R, -THETA2)
    pD = pSol(-R, -THETA2)

    return [pA, pB, pC, pD]

def filter_sols(sols, pan_mode):
    pan_forward = None
    pan_backward = None
    for sol in sols:
        if sol[0] > 0 and sol[2] < 0:
            pan_forward = sol
        if sol[0] < 0 and sol[2] > 0:
            pan_backward = sol
    
    if pan_mode == 'pan_forward':
        return pan_forward
    elif pan_mode == 'pan_backward':
        return pan_backward
    
    raise Exception('Not a valid pan mode: must be either pan_forward or pan_backward')

def get_sol(x, y, z, pan_mode = 'pan_forward'):
    sols = get_sols(x, y, z)
    return filter_sols(sols, pan_mode)