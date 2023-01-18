'''hw6p4sol.py

   This is the solution code for HW6 Problem 4.

   This uses the trajectory from Problem 3, updating the inverse
   kinematics to handle the redundancy of the 7~DOF arm.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hwsols.GeneratorNode      import GeneratorNode
from hwsols.KinematicChain     import KinematicChain
from hwsols.TransformHelpers   import *


#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.pleft  = np.array([ 0.3, 0.5, 0.15]).reshape((-1,1))
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))
        self.Rleft  = Rotx(-np.pi/2) @ Roty(-np.pi/2)
        self.Rleft  = Rotz( np.pi/2) @ Rotx(-np.pi/2)
        self.Rright = Reye()

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        # Pick the mode
        # self.mode = 'Pseudo'          # Pseudo Inverse only
        # self.mode = 'Primary'         # Additoinal primary task
        # self.mode = 'Goal'            # Secondary task as goal
        self.mode = 'Repulsion'       # Secondary task as repulsion
        node.get_logger().info("Mode = " + self.mode)

        # Also zero the task error (depending on the number of tasks).
        if self.mode == 'Primary':
            self.err = np.zeros((7,1))
        else:
            self.err = np.zeros((6,1))

        # Pick the convergence bandwidth.
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4',
                'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Decide which phase we are in:
        if t < 3:
            # Approach movement:
            (s0, s0dot) = spline(t, 3, 0, 1)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros((3,1))

        else:
            # Cyclic (sinusoidal) movements, after the first 3s.
            s    =               np.cos(np.pi/2.5 * (t-3))
            sdot = - np.pi/2.5 * np.sin(np.pi/2.5 * (t-3))

            # Use the path variables to compute the position trajectory.
            pd = np.array([-0.3*s    , 0.5, 0.75-0.6*s**2  ]).reshape((3,1))
            vd = np.array([-0.3*sdot , 0.0,     -1.2*s*sdot]).reshape((3,1))

            # Choose one of the following methods to compute orientation.
            if False:
                alpha    = - np.pi/4 * (s-1)
                alphadot = - np.pi/4 * sdot

                Rd = Rotx(-alpha) @ Roty(-alpha)
                wd = (- ex() - Rotx(-alpha) @ ey()) * alphadot

            elif False:
                alpha    = - np.pi/4 * (s-1)
                alphadot = - np.pi/4 * sdot
                
                Rd = Rotz(alpha) @ Rotx(-alpha)
                wd = (ez() - Rotz(alpha) @ ex()) * alphadot

            else:
                alpha    = - np.pi/3 * (s-1)
                alphadot = - np.pi/3 * sdot

                eleft = np.array([1, 1, -1]).reshape((3,1)) / np.sqrt(3)
                Rd    = Rote(eleft, -alpha)
                wd    = - eleft * alphadot


        # Grab the last joint value and task error.
        q   = self.q
        err = self.err

        # Compute the inverse kinematics according to the sub-problem:
        if self.mode == 'Pseudo':
            # Part (a): pseudo-inverse
            J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
            xdot = np.vstack((vd, wd))

            Jinv = np.linalg.pinv(J)
            qdot = Jinv @ (xdot + self.lam * err)

            q    = q + dt * qdot
            self.chain.setjoints(q)

            err  = np.vstack((ep(pd, self.chain.ptip()),
                              eR(Rd, self.chain.Rtip())))

        elif self.mode == 'Primary':
            # Part (b): additional primary task
            J7   = np.array([0.5, 0, 1, 0, 0, 0, 0]).reshape((1,7))
            vd7  = 0

            J    = np.vstack((self.chain.Jv(),self.chain.Jw(), J7))
            xdot = np.vstack((vd, wd, vd7))

            Jinv = np.linalg.pinv(J)
            qdot = Jinv @ (xdot + self.lam * err)

            q    = q + dt * qdot
            self.chain.setjoints(q)

            xd7  = 0
            x7   = 0.5*q[0,0] + q[2,0]
            err  = np.vstack((ep(pd, self.chain.ptip()),
                              eR(Rd, self.chain.Rtip()),
                              xd7-x7))

        elif self.mode == 'Goal':
            # Part (c): secondary task as goal
            lams  = 10.0
            qc    = np.radians(np.array([-45,-45,90,-90,0,0,0]).reshape((7,1)))
            qsdot = lams * (qc - q)

            J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
            xdot = np.vstack((vd, wd))

            Jinv = np.linalg.pinv(J)
            qdot = Jinv @ (xdot + self.lam * err) + qsdot - Jinv @ (J @ qsdot)

            q    = q + dt * qdot
            self.chain.setjoints(q)

            err  = np.vstack((ep(pd, self.chain.ptip()),
                              eR(Rd, self.chain.Rtip())))

        elif self.mode == 'Repulsion':
            # Part (d): secondary task as repulsion
            c      = 10
            qs0dot =         q[0,0]           * c / (q[0,0]**2 + q[1,0]**2)
            qs1dot = max(abs(q[0,0]), q[1,0]) * c / (q[0,0]**2 + q[1,0]**2)
            qsdot  = np.array([qs0dot, qs1dot, 0, 0, 0, 0, 0]).reshape((7,1))

            J    = np.vstack((self.chain.Jv(),self.chain.Jw()))
            xdot = np.vstack((vd, wd))

            Jinv = np.linalg.pinv(J)
            qdot = Jinv @ (xdot + self.lam * err) + qsdot - Jinv @ (J @ qsdot)

            q    = q + dt * qdot
            self.chain.setjoints(q)

            err  = np.vstack((ep(pd, self.chain.ptip()),
                              eR(Rd, self.chain.Rtip())))

        else:
            raise Exception("Unknown Redundancy Mode")

        # Save the joint value and task error for the next cycle.
        self.q   = q
        self.err = err

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
