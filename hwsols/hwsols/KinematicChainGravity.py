"""KinematicChainGravity.py

   This is the solution code for Kinematic Chains w/Gravity (HW7 Problem 1).

   chain = KinematicChain(node, basefame, tipframe, expectedjointnames)

      Initialize the kinematic chain, read from the URDF message on
      the topic /robot_description, sent from the robot_state_publisher.
      Extract the joints moving from the baseframe to the tipframe.
      Expect the active joints to match the given names.

   chain.name               Name of the robot
   chain.dofs               Active DOFS (# of active joints)
   chain.joints             List of joints (active and fixed) from URDF XML

   chain.data               Data for the active joints:
   chain.data.type[DOFS]    List of types per joint
   chain.data.e[DOFS]       List of joint axes w.r.t. world per joint
   chain.data.T[DOFS]       List of T matrices w.r.t. world per joint
   chain.data.Ttip          T matrix for tip w.r.t. world
   chain.data.grav          Vector of gravity torques

   chain.setjoints(q)       Update the chain.data to the joint position (q)

   chain.ptip()             Determine/return the tip position
   chain.Rtip()             Determine/return the tip orientation
   chain.Ttip()             Determine/return the tip transform
   chain.g()                Return the gravity vector
   chain.Jv()               Determine/return the tip translational Jacobian
   chain.Jw()               Determine/return the tip rotational Jacobian

   (ptip, Rtip, Jv, Jw) = chain.fkin(q)

      Update the kinematic chain.data and report the results.


   Note, internally, the URDF XML file is converted into a python list
   of Joint elements.  These contain

   Joint            URDF <joint> element
   Joint.name       Name of the joint
   Joint.parent     Parent link/frame
   Joint.child      Child  link/frame
   Joint.type       Type = 'fixed'                    fixed
                           'continuous', 'revolute'   rotational
                           'prismatic'                translational
   Joint.origin     Joint's fixed T matrix shift
   Joint.axis       Joint's axis of movement (if applicable)


   Node:        /kintest or as given
   Subscribe:   /robot_description      std_msgs/String

"""

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot

from hwsols.TransformHelpers import *


#
#   Kinematic Chain Data Class
#
class KinematicChainData:
    def __init__(self):
        self.type = []  # List of 'revolute' or 'prismatic'
        self.e = []  # List of 3x1 joint axes
        self.T = []  # List of 4x4 transforms
        self.Ttip = None  # 4x4 tip transform
        self.grav = []  # Nx1 Vector of gravity torques


#
#   Kinematic Chain Class
#
class KinematicChain:
    # Helper functions for printing info and errors.
    def info(self, string):
        self.node.get_logger().info("KinematicChain: " + string)

    def error(self, string):
        self.node.get_logger().error("KinematicChain: " + string)
        raise Exception(string)

    # Initialization.
    def __init__(self, node, baseframe, tipframe, expectedjointnames):
        # Store the node (for the printing functions).
        self.node = node

        # Create a temporary subscriber to receive the URDF.  We use
        # the TRANSIENT_LOCAL durability, so that we see the last
        # message already published (if any).
        self.info("Waiting for the URDF to be published...")
        self.urdf = None

        def cb(msg):
            self.urdf = msg.data

        topic = "/robot_description"
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        sub = node.create_subscription(String, topic, cb, quality)
        while self.urdf is None:
            rclpy.spin_once(node)
        node.destroy_subscription(sub)

        # Convert the URDF string into a Robot object and report.
        robot = Robot.from_xml_string(self.urdf)
        self.name = robot.name
        self.info("URDF Robot '%s'" % self.name)

        # Convert the URDF string into a Robot object and parse to
        # create the list of joints from the base frame to the tip
        # frame.  Search backwards, as this could be a tree structure.
        # Meaning while a parent may have multiple children, every
        # child has only one parent!  That makes the chain unique.
        self.joints = []
        self.inertias = []
        frame = tipframe
        while frame != baseframe:
            joint = next((j for j in robot.joints if j.child == frame), None)
            link = next((l for l in robot.links if l.name == frame), None)
            if link is None:
                self.error("Unable find link '%s'" % frame)
            if joint is None:
                self.error("Unable find joint connecting to '%s'" % frame)
            if joint.parent == frame:
                self.error("Joint '%s' connects '%s' to itself" % (joint.name, frame))
            self.joints.insert(0, joint)
            self.inertias.insert(0, link.inertial)
            frame = joint.parent

        # Report we found.
        self.dofs = sum(1 for j in self.joints if j.type != "fixed")
        self.info(
            "%d total joints in URDF, %d active DOFs:" % (len(self.joints), self.dofs)
        )
        dof = 0
        for i in range(len(self.joints)):
            joint = self.joints[i]
            inertia = self.inertias[i]
            if joint.type == "fixed":
                string = "Joint #%d fixed      " % i
            elif joint.type == "continuous" or joint.type == "revolute":
                string = "Joint #%d rot DOF #%d " % (i, dof)
                dof = dof + 1
            elif joint.type == "prismatic":
                string = "Joint #%d lin DOF #%d " % (i, dof)
                dof = dof + 1
            else:
                self.error(
                    "Joint '%s' has unknown type '%s'" % (joint.name, joint.type)
                )
            string += "%-20s" % ("'" + joint.name + "'")
            if inertia is not None:
                string += "mass %6.3fkg" % inertia.mass
            else:
                string += "unknown mass"
            self.info(string)

        # Confirm this matches the expectation
        jointnames = [j.name for j in self.joints if j.type != "fixed"]
        if jointnames != list(expectedjointnames):
            self.error(
                "Chain does not match the expected names: " + str(expectedjointnames)
            )

        # And pre-compute the kinematic chain data at the zero position.
        self.setjoints(np.zeros((self.dofs, 1)))

    # Update the joint positions.  This recomputes the chain data values!
    def setjoints(self, q):
        # Check the number of joints
        if len(q) != self.dofs:
            self.error(
                "Number of joint angles (%d) does not match URDF (%d)",
                len(q),
                self.dofs,
            )

        # Remove any past data.
        self.data = KinematicChainData()

        # Initialize the T matrix to walk up the chain, w.r.t. world frame.
        T = np.eye(4)

        # Initialize the gravoty torques.
        grav = np.zeros((self.dofs, 1))

        # Walk the chain, one URDF <joint> entry at a time, being
        # 'fixed' (just a fixed transform), 'continuous'/'revolute'
        # (both rotational, the latter with joint limits which we
        # ignore), or 'prismatic'.  By design, the URDF entries are
        # only the step-by-step transformations.  That is, the
        # information is *not* in world frame.  We have to append to
        # the chain...
        dof = 0
        for (joint, inertia) in zip(self.joints, self.inertias):
            if joint.type == "fixed":
                # Just append the fixed transform
                T = T @ T_from_URDF_origin(joint.origin)

            elif (joint.type == "continuous") or (joint.type == "revolute"):
                # Grab the joint axis in the local frame.
                elocal = e_from_URDF_axis(joint.axis)

                # First append the fixed transform, then the rotational
                # transform.  The joint angle comes from q-vector.
                T = T @ T_from_URDF_origin(joint.origin)
                T = T @ T_from_Rp(Rote(elocal, q[dof]), pzero())

                # Compute the joint axis w.r.t. world frame.
                e = R_from_T(T) @ elocal

                # Save the transform and advance the active DOF counter.
                self.data.type.append("revolute")
                self.data.e.append(e)
                self.data.T.append(T)
                dof += 1

            elif joint.type == "prismatic":
                # Grab the joint axis in the local frame.
                elocal = e_from_URDF_axis(joint.axis)

                # First append the fixed transform, then the translational
                # transform.  The joint displacement comes from q-vector.
                T = T @ T_from_URDF_origin(joint.origin)
                T = T @ T_from_Rp(Reye(), elocal * q[dof])

                # Compute the joint axis w.r.t. world frame.
                e = R_from_T(T) @ elocal

                # Save the transform and advance the active DOF counter.
                self.data.type.append("prismatic")
                self.data.e.append(e)
                self.data.T.append(T)
                dof += 1

            else:
                # There shouldn't be any other types...
                self.error("Unknown Joint Type: %s", joint.type)

            # Also look at the child link to get the mass and gravity.
            # Proceed if there is an inertial element.
            if inertia is not None:
                # Check whether to shift to the child link's center of mass.
                if inertia.origin is not None:
                    Tlink = T @ T_from_URDF_origin(inertia.origin)
                else:
                    Tlink = T

                # Extract the mass times g and center of mass location.
                mg = inertia.mass * 9.81
                pc = p_from_T(Tlink)

                # Add all the torques (below the current dof).  Note
                # the gravity vector is vertically up in world frame,
                # hence we simply take the 3rd element of the vector.
                for k in range(dof):
                    grav[k] += (
                        mg
                        * np.cross(
                            self.data.e[k], pc - p_from_T(self.data.T[k]), axis=0
                        )[2]
                    )

        # Also save the tip transform and gravity vector.
        self.data.Ttip = T
        self.data.grav = grav

    # Extract the position/orientation data from the already computed
    # kinematic chain data in self.data!
    def ptip(self):
        return p_from_T(self.data.Ttip)

    def Rtip(self):
        return R_from_T(self.data.Ttip)

    def Ttip(self):
        return self.data.Ttip

    # Extract the gravity vector.
    def g(self):
        return self.data.grav

    # Extract the Jacobian data.
    def Jv(self):
        J = np.zeros((3, self.dofs))
        for dof in range(self.dofs):
            if self.data.type[dof] == "revolute":
                dp = p_from_T(self.data.Ttip) - p_from_T(self.data.T[dof])
                J[:, dof : dof + 1] = cross(self.data.e[dof], dp)
            else:
                J[:, dof : dof + 1] = self.data.e[dof]
        return J

    def Jw(self):
        J = np.zeros((3, self.dofs))
        for dof in range(self.dofs):
            if self.data.type[dof] == "revolute":
                J[:, dof : dof + 1] = self.data.e[dof]
            else:
                J[:, dof : dof + 1] = np.zeros((3, 1))
        return J

    # All-in-one call.
    def fkin(self, q):
        # Compute the chain data.
        self.setjoints(q)

        # Extract and return the data.
        return (self.ptip(), self.Rtip(), self.Jv(), self.Jw())


#
#   Main Code
#
#   This simply tests the kinematic chain and associated calculations!
#
def main(args=None):
    # Initialize ROS and the node.
    rclpy.init(args=args)
    node = Node("kintest")

    # Set up the kinematic chain object  with the expected joints.
    jointnames = ["theta1", "theta2", "theta3", "theta4"]
    chain = KinematicChain(node, "world", "tip", jointnames)

    # Define the test cases.
    qtest = [
        np.radians(np.array([0, 0, 0, 0]).reshape((4, 1))),
        np.radians(np.array([0, 0, 90, 0]).reshape((4, 1))),
        np.radians(np.array([0, 90, 0, 0]).reshape((4, 1))),
        np.radians(np.array([90, 90, 90, 90]).reshape((4, 1))),
        np.radians(np.array([45, 45, 45, 45]).reshape((4, 1))),
        np.radians(np.array([30, -60, 30, -60]).reshape((4, 1))),
    ]

    # Test each case.
    for q in qtest:
        # Compute the data.
        chain.setjoints(q)

        # Print the test case.
        np.set_printoptions(precision=3, suppress=True)
        print("Grav for: ", q.T)
        print("Torques : ", chain.g().T)
        print("")

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
