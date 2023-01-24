"""hw4p3.py

   This is the solution code for HW4 Problem 3.

   This simply uses NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

"""

import numpy as np


#
#  Link Lengths
#
l1 = 1
l2 = 1


#
#  Forward Kinematics
#
def fkin(q):
    # Precompute the sin/cos values from 2D joint vector.
    sp = np.sin(q[0, 0])
    cp = np.cos(q[0, 0])
    s1 = np.sin(q[1, 0])
    c1 = np.cos(q[1, 0])
    s12 = np.sin(q[1, 0] + q[2, 0])
    c12 = np.cos(q[1, 0] + q[2, 0])

    # Calculate the tip position.
    x = np.array(
        [
            [-sp * (l1 * c1 + l2 * c12)],
            [cp * (l1 * c1 + l2 * c12)],
            [(l1 * s1 + l2 * s12)],
        ]
    )

    # Return the tip position as a numpy 3x1 column vector.
    return x


#
#  Jacobian
#
def Jac(q):
    # Precompute the sin/cos values from 2D joint vector.
    sp = np.sin(q[0, 0])
    cp = np.cos(q[0, 0])
    s1 = np.sin(q[1, 0])
    c1 = np.cos(q[1, 0])
    s12 = np.sin(q[1, 0] + q[2, 0])
    c12 = np.cos(q[1, 0] + q[2, 0])

    # Calculate the tip position.
    J = np.array(
        [
            [-cp * (l1 * c1 + l2 * c12), sp * (l1 * s1 + l2 * s12), sp * l2 * s12],
            [-sp * (l1 * c1 + l2 * c12), -cp * (l1 * s1 + l2 * s12), -cp * l2 * s12],
            [0, (l1 * c1 + l2 * c12), l2 * c12],
        ]
    )

    # Return the Jacobian as a numpy 3x3 matrix.
    return J


#
#  Main Code
#
def main():
    # Set the joint cooordinates.  Explicity make it a column vector
    # by writing it as a list of lists.
    q = np.array([[np.radians(30)], [np.radians(30)], [np.radians(60)]])

    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)
    print("TEST CASE:")
    print("q:\n", q)
    print("fkin(q):\n", fkin(q))
    print("Jac(q):\n", Jac(q))


if __name__ == "__main__":
    main()
