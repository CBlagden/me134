import numpy as np

def get_sols(l1, l2, x, y, z, x_off, y_off, z_off):
    # offset goal position to be described in frame of middle joint
    x -= x_off
    y -= y_off
    z -= z_off

    R = np.sqrt(x * x + y * y)
    THETA2 = np.arccos((x * x + y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2))

    def pSol(r, theta2):
        thetapan = np.arctan2(x / r, -y / r)
        theta1 = np.arctan2(z, r) - np.arctan2(
            l1 + l2 * np.cos(theta2), l2 * np.sin(theta2)
        )
        return np.array([thetapan, theta1, theta2])

    pA = pSol(R, THETA2)
    pB = pSol(-R, THETA2)
    pC = pSol(R, -THETA2)
    pD = pSol(-R, -THETA2)

    return [pA, pB, pC, pD]