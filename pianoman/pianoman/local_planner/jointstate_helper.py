# jointstate_helper.py
# ME 134 Team Penguinos
#
# Class to hold all the joint state information compactly

import numpy as np

from pianoman.utils.KinematicChain import KinematicChain

from sensor_msgs.msg    import JointState


class JointStateHelper:
    def __init__(self, jointnames, L_idx, R_idx, gripper_idx):
        # save the indicies
        self.jointnames = jointnames.copy()
        self.L_idx = L_idx.copy()
        self.R_idx = R_idx.copy()
        self.gripper_idx = gripper_idx.copy()
        self.joint_idx = [0, 6, 7, 8, 2, 3, 4] # base, left, right
        # self.joint_idx = list(range(len(self.jointnames)))
        # for gi in self.gripper_idx:
        #     # self.joint_idx.remove(gi)

        # make fkin objects
        self.chain_L = KinematicChain("base_link", "L_tip_link", 'L')
        self.chain_R = KinematicChain("base_link", "R_tip_link", 'R')

        # placeholder variables for joint state measurements
        self.q_measured = None
        self.qdot_measured = None
        self.eff_measured = None


    def update_measurements(self, msg: JointState):
        self.q_measured = np.array(msg.position).reshape([9,1])
        self.qdot_measured = np.array(msg.velocity).reshape([9,1])
        self.eff_measured = np.array(msg.effort).reshape([9,1])

    # return the joint state values
    def get_gripper_measured(self):
        # [L, R]^T
        return [self.q_measured[self.gripper_idx], self.qdot_measured[self.gripper_idx], self.eff_measured[self.gripper_idx]]

    def get_joints_measured(self):
        # [L, R]^T
        return [self.q_measured[self.joint_idx], self.qdot_measured[self.joint_idx], self.eff_measured[self.joint_idx]]

    def get_all_measured(self):
        return [self.q_measured, self.qdot_measured, self.eff_measured]

    # Compute the 6 x 9 Jacobian
    def fkin(self, q_measured = None):
        if (q_measured is None):
            q_measured = self.q_measured
            q_measured_L = q_measured[self.L_idx]
            q_measured_R = q_measured[self.R_idx]
            assert self.q_measured is not None, "Call update_measurements before calling fkin"
        elif (len(q_measured) == 7):
            q_measured_L = q_measured[0:4, :]
            q_measured_R = q_measured[[0,4,5,6]]
        elif (len(q_measured) == 4):
            # default to left only
            q_measured_L = q_measured[0:4, :]
            q_measured_R = np.array([0.0, 0.0, 0.0, 0.0])

        # compute the two chains
        [p_L, R_L, Jv_L, Jw_L] = self.chain_L.fkin(q_measured_L)
        [p_R, R_R, Jv_R, Jw_R] = self.chain_R.fkin(q_measured_R)
        p = np.vstack([p_L, p_R])

        # Fill in the Jacobian: 6 x 7
        Jv = np.zeros([6, 7])
        Jw = np.zeros([6, 7])

        Jv[:, 0] = np.vstack([Jv_L, Jv_R])[:, 0]
        Jv[0:3, 1:4] = Jv_L[:, 1:]
        Jv[3:6, 4:7] = Jv_R[:, 1:]

        Jw[:, 0] = np.vstack([Jw_L, Jw_R])[:, 0]
        Jw[0:3, 1:4] = Jw_L[:, 1:]
        Jw[3:6, 4:7] = Jw_R[:, 1:]

        return [p, R_L, R_R, Jv, Jw]

    # message helper
    def to_msg(self, t, q, qdot, eff, grip_q):
        # concatincate q with grip_q
        poscmd = 9 * [float("NaN")]
        velcmd = 9 * [float("NaN")]
        effcmd = 9 * [float("NaN")]

        for iq, icmd in enumerate(self.joint_idx):
            poscmd[icmd] =    q[iq]
            velcmd[icmd] = qdot[iq]
            effcmd[icmd] =  eff[iq]

        for iq, icmd in enumerate(self.gripper_idx):
            poscmd[icmd] = grip_q[iq]
            velcmd[icmd] = float("NaN")
            effcmd[icmd] = float("NaN")

        cmdmsg = JointState()
        cmdmsg.header.stamp = t.to_msg()
        cmdmsg.name = self.jointnames
        cmdmsg.position = poscmd
        cmdmsg.velocity = velcmd
        cmdmsg.effort = effcmd

        return cmdmsg

    
