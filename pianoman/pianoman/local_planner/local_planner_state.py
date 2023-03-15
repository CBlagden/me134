# local_planner.py
# ME 134 Team Penguinos
#
# Node to direct the robot's arms to a target end effector position.
# 
# SUBSCRIBES:
# /robot_state      me134_interfaces.msg/StateStamped
# /keyboarddetector/keyboard_point  geometry_msgs.msg/Point
# /note_cmd         me134_interfaces.srv/NoteCmdStamped
# 
# PUBLISHES:
# /joint_commands   sensor_msgs.msg/JointState
# /goal_marker      visualization_msgs.msg/Marker

import rclpy
from rclpy.node import Node

import scipy.linalg
import numpy as np

from pianoman.local_planner.states import State
from pianoman.utils.rviz_helper import create_pt_marker
from pianoman.utils.Segments import Goto5, Hold, Stay
import pianoman.utils.midi_helper as midi_helper
from pianoman.utils.TransformHelpers import Rotz, R_from_quat
from pianoman.utils.StateClock import StateClock

from pianoman.local_planner.jointstate_helper import JointStateHelper

from std_msgs.msg import Empty, String
from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import Pose
from visualization_msgs.msg import Marker
from me134_interfaces.msg import PoseTwoStamped
from me134_interfaces.srv import NoteCmdStamped, PosCmdStamped

RATE = 200.0 # Hz
LAM = 40

# P_BASE_WORLD = np.array([-0.018, 0.69, 0.0]).reshape([3, 1]) # m
P_BASE_WORLD = np.array([-0.018 + 0.03, 0.69 + 0.01, 0.0]).reshape([3, 1]) # m\
GRIPPER_OPEN_EFF = 1.0
GRIPPER_CLOSED_EFF = -2.5

L_IDX = [0, 6, 7, 8]
R_IDX = [0, 2, 3, 4]
GRIP_IDX = [5, 1]
JOINT_NAMES = ["base_joint", \
                "R_gripper_joint", "R_pan_joint", "R_lower_joint", "R_upper_joint", \
                "L_gripper_joint", "L_pan_joint", "L_lower_joint", "L_upper_joint"]

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        # self.get_logger().info("Local planner started.")

        ## Class variables
        # state variable
        self.state = State.default()
        # For updating if hit something:
        self.eff_cmd_stamp = -1.0
        self.eff_cmd = np.array(9 * [0.]).reshape([9,1])
        
        self.pos_cmd_stamp = -1.0
        self.pos_cmd = np.array(9 * [0.]).reshape([9,1])

        self.position_cmd = np.array(6 * [0.]).reshape([6,1])
        # get initial joint position sensor reading
        self.fbk = JointStateHelper(JOINT_NAMES, L_IDX, R_IDX, GRIP_IDX)
        self.fbk.update_measurements(self.grabfbk())

        self.kb_pos = None
        self.kb_rot = np.eye(3)

        q0, _, _ = self.fbk.get_joints_measured()
        self.qd = q0

        # general
        self.playsplines = [None, None]
        self.grip_effcmd = [GRIPPER_OPEN_EFF, GRIPPER_OPEN_EFF]
        self.counter = 0

        # state clocks
        self.play_clock_L = None
        self.play_clock_R = None
        # time variables
        self.tstart = None

        ## Publishers
        self.pub_jtcmd = self.create_publisher(\
                JointState, '/joint_commands', 10)
        self.pub_goalmarker_L = self.create_publisher(\
                Marker, '/goal_marker_L', 10)
        self.pub_goalmarker_R = self.create_publisher(\
                Marker, '/goal_marker_R', 10)
        self.pub_pose = self.create_publisher(PoseTwoStamped, '/pose', 10)
        self.pub_pose_err = self.create_publisher(PoseTwoStamped, '/pose_error', 10)

        ## Subscribers
        self.sub_jtstate = self.create_subscription(\
                JointState, '/joint_states', self.cb_jtstate, 10)
        self.sub_kb_pos = self.create_subscription(\
                Pose, '/keyboarddetector/keyboard_point', self.cb_update_kb_pos, 10)
        self.sub_pull_kb = self.create_subscription(\
                Empty, '/pull_keyboard', self.cb_pull_kb, 10)
        self.sub_songcmd = self.create_subscription(
                String, '/song_cmd', self.cb_songcmd, 10)

        ## Services
        self.srv_pointcmd = self.create_service(\
                PosCmdStamped, '/point_cmd', self.cb_pointcmd)
        self.srv_notecmd = self.create_service(\
                NoteCmdStamped, '/note_cmd', self.cb_notecmd)

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)

    # Main motor timer
    def cb_sendcmd(self):
        # Send commands based on the current state
        if (self.state == State.FLOAT):
            # Chill in floating mode
            poscmd = 7 * [float("NaN")]
            velcmd = 7 * [float("NaN")]
            effcmd = self.gravity()
            self.grip_effcmd = [GRIPPER_OPEN_EFF, GRIPPER_OPEN_EFF]

            q, _, _ = self.fbk.get_joints_measured()
            self.qd = q
            self.playsplines = [None, None]
            self.jointsplines = None

            cmdmsg = self.fbk.to_msg(self.get_clock().now(), poscmd, velcmd, effcmd, self.grip_effcmd)
            self.pub_jtcmd.publish(cmdmsg)


        if (self.state == State.PLAY):
            # get current time
            t = self.get_clock().now()
            dt = 1/RATE

            # Default to floating mode
            poscmd = 7 * [float("NaN")]
            velcmd = 7 * [float("NaN")]
            effcmd = self.gravity()

            # default to no spline in L or R
            cursplines = [None,None]
            deltats = [None, None]

            # self.playsplines: [list of left splines, list of right splines]
            splinelist_L = self.playsplines[0]
            if (splinelist_L):
                cursplines[0] = splinelist_L[0]
                deltats[0] = self.play_clock_L.t_since_start(t, rostime=True)

            splinelist_R = self.playsplines[1]
            if (splinelist_R):
                cursplines[1] = splinelist_R[0]
                deltats[1] = self.play_clock_R.t_since_start(t, rostime=True)
            # self.get_logger().info(f"{deltats}")

            # check for spline completion
            if (cursplines[0]):
                if (cursplines[0].completed(deltats[0])):
                    # remove spline
                    self.play_clock_L.restart_with_deltat(t, deltats[0] - cursplines[0].T, rostime=True)
                    splinelist_L.pop(0)
                    # ignore spline for this timestep (TODO: okay?)
                    cursplines[0] = None

            if (cursplines[1]):
                if (cursplines[1].completed(deltats[1])):
                    # remove spline
                    self.play_clock_R.restart_with_deltat(t, deltats[1] - cursplines[1].T, rostime=True)
                    splinelist_R.pop(0)
                    # ignore spline for this timestep (TODO: okay?)
                    cursplines[1] = None

            # only do something if L or R contains a command
            if ((cursplines[0]) or (cursplines[1])):
                # check the spaces
                if (cursplines[0]):
                    spaceL = cursplines[0].get_space()
                else:
                    # just a placeholder for logic
                    spaceL = cursplines[1].get_space()
                if (cursplines[1]):
                    spaceR = cursplines[1].get_space()
                else:
                    # just a placeholder for logic
                    spaceR = cursplines[0].get_space()
                
                # consider task space moves
                if (spaceL == 'task' and spaceR == 'task') or \
                   (spaceL == 'keyboard' and spaceR == 'keyboard'):
                    if (spaceL == 'keyboard' and spaceR == 'keyboard'):
                        # Get the keyboard stuff
                        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
                        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

                    # start with forward kinematics
                    [xcurr, _, _, Jv, _] = self.fbk.fkin(self.qd)

                    # evaluate the splines
                    # CASE 1: BOTH ARMS HAVE COMMANDS
                    if (cursplines[0] and cursplines[1]):
                        num_arms = 2
                        [xd_L, xd_dot_L] = cursplines[0].evaluate(deltats[0])
                        xd_L = xd_L.reshape([3,1])
                        xd_dot_L = xd_dot_L.reshape([3,1])
                        if (spaceL == 'keyboard'):
                            # convert back to task space
                            xd_L = self.kb_to_robot_frame(xd_L, R_kb_world, delta)
                            xd_dot_L = self.kb_to_robot_frame(xd_dot_L, R_kb_world, delta)

                        [xd_R, xd_dot_R] = cursplines[1].evaluate(deltats[1])
                        xd_R = xd_R.reshape([3,1])
                        xd_dot_R = xd_dot_R.reshape([3,1])
                        if (spaceL == 'keyboard'):
                            # convert back to task space
                            xd_R = self.kb_to_robot_frame(xd_R, R_kb_world, delta)
                            xd_dot_R = self.kb_to_robot_frame(xd_dot_R, R_kb_world, delta)

                        xd = np.vstack([xd_L, xd_R])
                        xd_dot = np.vstack([xd_dot_L, xd_dot_R])
                        qd = self.qd

                        # Save xd for publishing state error
                        self.position_cmd = xd.copy().reshape([6,1])

                        # does not change Jv or xcurr
                    # CASE 2: ONLY LEFT ARM HAS COMMANDS
                    elif (cursplines[0]):
                        num_arms = 1
                        [xd, xd_dot] = cursplines[0].evaluate(deltats[0])
                        xd = xd.reshape([3,1])
                        xd_dot = xd_dot.reshape([3,1])
                        if (spaceL == 'keyboard'):
                            # convert back to task space
                            xd = self.kb_to_robot_frame(xd, R_kb_world, delta)
                            xd_dot = self.kb_to_robot_frame(xd_dot, R_kb_world, delta)
                            # Save xd for publishing state error
                            self.position_cmd = np.vstack([xd.copy().reshape([3,1]), np.zeros([3,1])])

                        qd = self.qd[0:4, :]

                        # remove the last three columns of Jv (they correspond to the right arm)
                        Jv = Jv[0:3, 0:4]
                        xcurr = xcurr[0:3,:]  # pull out left arm positions

                    # CASE 3: ONLY RIGHT ARM HAS COMMANDS
                    elif (cursplines[1]):
                        num_arms = 1
                        [xd, xd_dot] = cursplines[1].evaluate(deltats[1])
                        xd = xd.reshape([3,1])
                        xd_dot = xd_dot.reshape([3,1])
                        if (spaceR == 'keyboard'):
                            # convert back to task space
                            xd = self.kb_to_robot_frame(xd, R_kb_world, delta)
                            xd_dot = self.kb_to_robot_frame(xd_dot, R_kb_world, delta)
                            # Save xd for publishing state error
                            self.position_cmd = np.vstack([np.zeros([3,1]), xd.copy().reshape([3,1])])

                        qd = self.qd[[0,4,5,6], :]

                        # remove columns 1, 2, 3 of Jv (they correspond to the left arm)
                        Jv = Jv[3:6, [0, 4, 5, 6]]
                        xcurr = xcurr[3:6,:]  # pull out right arm positions


                    # Weighted pseudoinverse for singularity prevention
                    # GAM2 = 0.01
                    GAM2 = np.ones((1 + 3 * num_arms)) * 0.01
                    # GAM[0] = 0.0005
                    # GAM[1:] = 0.01
                    # W = np.diag(GAM) # TODO: double check
                    W = np.eye(1 + 3 * num_arms)
                    Jv_pinv = np.linalg.pinv(W @ Jv.T @ Jv @ W + np.diag(GAM2)) @ W @ Jv.T
                    # Jv_pinv = np.linalg.pinv(Jv.T @ Jv + GAM2*np.eye(1 + 3*num_arms)) @ Jv.T
                    # Jv_pinv = np.linalg.pinv(Jv) # UNCOMMENT TO REMOVE SINGULARITY PREVENTION

                    # if one of the singular values is too small, we are near a singularity
                    # and so should return back to floating mode
                    SINGULARITY_THRESHOLD = 1e-4 # DON'T LOWER THIS IT'S SCARY!
                    s = scipy.linalg.svdvals(Jv)
                    if np.any(s < SINGULARITY_THRESHOLD):
                        self.state = State.FLOAT

                    
                    # get qdot with J qdot = xdot
                    ex = xd - xcurr
                    qd_dot = Jv_pinv @ (xd_dot + LAM * ex)

                    # Secondary task to keep pan joint aligned with keyboard orientation
                    # and elbow pointing up
                    q_sec_goal = np.zeros((1+3*num_arms, 1))
                    keyboardOrientation = np.arctan2(self.kb_rot[1, 0], self.kb_rot[0, 0])
                    
                    q_sec_goal[0, 0] = keyboardOrientation # base pan joint should track keyboard orientation

                    l_base = 8.0
                    l_pan = 1.5
                    l_lower = 0.3
                    l_upper = 0.3

                    if num_arms == 1: # TODO: Check that this generalizes to right only commands
                        lambda_s = np.array([l_base, l_pan, l_lower, l_upper]).reshape((4, 1))
                        # we only have a left arm
                        q_sec_goal[2, 0] = 0.9 # left lower joint
                        q_sec_goal[3, 0] = -1.2 # left upper joint
                        
                        qd_sec = lambda_s * (q_sec_goal - qd)

                        # qd_sec[1, 0] = 0.0 # left pan joint stays free

                    else:
                        # we have two arms, yay us!  
                        lambda_s = np.array([l_base, l_pan, l_lower, l_upper, l_pan, l_lower, l_upper]).reshape((7, 1))
                        q_sec_goal[2, 0] = 0.9 # right lower joint
                        q_sec_goal[3, 0] = -1.2 # right upper joint
                        q_sec_goal[5, 0] = -0.9 # left lower joint
                        q_sec_goal[6, 0] = 1.2 # left upper joint
                        
                        qd_sec = lambda_s * (q_sec_goal - qd)
                        # qd_sec[1, 0] = 0.0 # right pan joint stays free
                        # qd_sec[4, 0] = 0.0 # left pan joint stays free


                    # Inverse kinematics!
                    qd_dot = Jv_pinv @ (xd_dot + LAM * ex)

                    # Map secondary tasks to the null space
                    qd_dot += (np.eye(1 + 3 * num_arms) - Jv_pinv @ np.linalg.pinv(Jv_pinv)) @ qd_sec # COMMENT OUT TO REMOVE SECONDARY TASKS

                    # get desired q from Euler integration with qdot
                    qd += qd_dot * dt
                    if (cursplines[0] and cursplines[1]):
                        self.qd = qd
                    elif (cursplines[0]):
                        self.qd[0:4] = qd
                    elif (cursplines[1]):
                        self.qd[[0,4,5,6]] = qd


                    # Combine commands!
                    if (cursplines[0] and cursplines[1]):
                        poscmd = list(self.qd.reshape([7]))
                        velcmd = list(qd_dot.reshape([7]))
                    elif (cursplines[0]):
                        poscmd = list(self.qd[:4, 0].reshape([4])) + 3 * [float("NaN")]
                        velcmd = list(qd_dot.reshape([4])) + 3 * [float("NaN")]
                    elif (cursplines[1]):
                        poscmd = 3 * [float("NaN")] + list(self.qd[[0,4,5,6], 0].reshape([4]))
                        velcmd = 3 * [float("NaN")] + list(qd_dot.reshape([4]))

                self.counter = 0
            else:
                self.counter += 1
                if (self.counter > 4):
                    # If no spline to run, return to floating mode
                    self.state = State.FLOAT
                    self.counter = 0
                

            # Publish!
            cmdmsg = self.fbk.to_msg(t, poscmd, velcmd, effcmd, self.grip_effcmd)
            self.pub_jtcmd.publish(cmdmsg)

            # save effort command and stamp
            self.eff_cmd = np.array(cmdmsg.effort).reshape([9,1])
            self.eff_cmd_stamp = cmdmsg.header.stamp.sec + 1e-9 * cmdmsg.header.stamp.nanosec

            # save position command and stamp
            self.pos_cmd = np.array(cmdmsg.position).reshape([9,1])
            self.pos_cmd_stamp = cmdmsg.header.stamp.sec + 1e-9 * cmdmsg.header.stamp.nanosec

    def cb_songcmd(self, msg):
        """
            Given a message with a path to a midi file, parses the file
            and plays the accompanying song
        """
        self.neutral_above_piano()
        filename = msg.data
        left_traj, right_traj = midi_helper.note_trajectories(filename,
                                                              left_bpm=25,
                                                              right_bpm=25)
        trajs = [left_traj, right_traj]
        # for i in range(len(trajs)):
        for i in [0, 1]:
            is_left = i == 0
            for j, (note, duration) in enumerate(trajs[i]):
                is_first = j == 0
                msg = NoteCmdStamped.Request()
                if is_left:
                    msg.note_left = note
                    msg.hold_time_left = duration
                    msg.cmd_left = True
                else:
                    msg.note_right = note
                    msg.hold_time_right = duration
                    msg.cmd_right = True

                msg.space = "keyboard"
                self.primitive_playnote(msg,
                                        left=is_left,
                                        space="keyboard",
                                        first=is_first)
        self.state = State.PLAY
        print(self.playsplines[0])
        print(self.playsplines[1])


    ## Callback functions
    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint positions
        self.fbk.update_measurements(msg)

        [p, _, _, _, _] = self.fbk.fkin()
        # self.get_logger().info(f"pL: {p[0,0]:.3f}, {p[1,0]:.3f}, {p[2,0]:.3f}")

        # Publish current pose
        pose_msg = PoseTwoStamped()
        p_L = p[:3, 0]
        p_R = p[3:, 0]

        def _make_pose(arr):
            p = Pose()
            p.position.x = arr[0]
            p.position.y = arr[1]
            p.position.z = arr[2]
            return p

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose_left = _make_pose(p_L)
        pose_msg.pose_right = _make_pose(p_R)

        self.pub_pose.publish(pose_msg)

        # publish pose error
        pose_err_msg = PoseTwoStamped()

        pose_err_msg.header.stamp = self.get_clock().now().to_msg()
        pose_err_msg.header.frame_id = "base_link"
        pose_err_msg.pose_left =  _make_pose(p_L - self.position_cmd[:3, 0])
        if (len(self.position_cmd) == 6):
            pose_err_msg.pose_right = _make_pose(p_R - self.position_cmd[3:, 0])

        self.pub_pose_err.publish(pose_err_msg) # TODO: CHECK THIS
        


        # In all states, check if effort is far from command
        # first check how old effort command is
        t_cur = self.get_clock().now().nanoseconds * 1e-9
        timeout_sec = 0.5 # s
        EFFTHRESH = 2.5 # Nm

        eff_measured = np.array(msg.effort).reshape([9,1])
        if (t_cur - self.eff_cmd_stamp > timeout_sec): 
            # Command is stale... don't compute (TODO: FIX)
            effdiff = eff_measured
        else:
            # command is recent; use the difference with measurement
            self.eff_cmd = np.nan_to_num(self.eff_cmd, nan=0) # replace NaN values
            effdiff = eff_measured - self.eff_cmd
            effdiff = effdiff[[0, 2, 3, 4, 6, 7, 8]] # TODO: incorporate gripper
            # self.get_logger().info(str((abs(effdiff))))

            # check for a state change
            # TODO: INCLUDE GRIPPER
            if (sum(abs(effdiff) > EFFTHRESH) > 0):
                if (self.state != State.FLOAT):
                    self.get_logger().info("Hit something! Returning to float mode.")
                    self.state = State.FLOAT

        # # In all states, check if position is far from command
        # # first check how old position command is
        # t_cur = self.get_clock().now().nanoseconds * 1e-9
        # timeout_sec = 0.5 # s
        # POSTHRESH = 0.1 # m

        # pos_measured = np.array(msg.position).reshape([9,1])
        # if (t_cur - self.pos_cmd_stamp > timeout_sec): 
        #     # Command is stale... don't compute (TODO: FIX)
        #     posdiff = pos_measured
        # else:
        #     # command is recent; use the difference with measurement
        #     self.pos_cmd = np.nan_to_num(self.pos_cmd, nan=0) # replace NaN values
        #     posdiff = pos_measured - self.pos_cmd
        #     posdiff = posdiff[[0, 2, 3, 4, 6, 7, 8]] # TODO: incorporate gripper
        #     # self.get_logger().info(str((abs(effdiff))))

        #     # check for a state change
        #     # TODO: INCLUDE GRIPPER
        #     if (sum(abs(posdiff) > POSTHRESH) > 0):
        #         if (self.state != State.FLOAT):
        #             self.get_logger().info("Hit something! Returning to float mode.")
        #             self.state = State.FLOAT


    def cb_update_kb_pos(self, msg: Pose):
        # technically z never changes but we save it just in case
        self.kb_pos = np.array([msg.position.x, msg.position.y, msg.position.z]).reshape([3,1])
        self.kb_rot = R_from_quat(np.array([msg.orientation.x,
                                            msg.orientation.y,
                                            msg.orientation.z,
                                            msg.orientation.w])).reshape((3, 3))

    # callback for /point_cmd
    def cb_pointcmd(self, msg, response):
        #
        # self.get_logger().info(f"Got point command: [{msg.goal_left.x}, {msg.goal_left.y}, {msg.goal_left.z}]")

        # reset spline list
        self.playsplines = [None, None]
        self.grip_effcmd = [float('NaN'), float('NaN')]

        if (msg.cmd_left):
            # Command only the left
            pd_final = [msg.goal_left.x, msg.goal_left.y, msg.goal_left.z]
            pd_final = np.array(pd_final).reshape([3,1])

            # get current position
            [pcur, _, _, _, _] = self.fbk.fkin()
            pcur = pcur[0:3, :]
            pcur = np.array(pcur).reshape([3,1])

            # publish the goal point
            markermsg = create_pt_marker(pd_final[0,0], pd_final[1,0], pd_final[2,0], self.get_clock().now().to_msg())
            self.pub_goalmarker_L.publish(markermsg)

            # Set qd to be actual q
            # # WARNING: DO NOT DO THIS FOR NOTES
            # q, _, _ = self.fbk.get_joints_measured()
            # self.qd = q[0:4, :]
            
            self.grip_effcmd[0] = GRIPPER_CLOSED_EFF
            self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)

            # add in the splines
            move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
            spline1 = Goto5(pcur, pd_final, move_time, space='task')
            spline2 = Stay(pd_final, space='task')
            if (self.playsplines[0] == None):
                self.playsplines[0] = [spline1]
            else:
                self.playsplines[0].append(spline1)
            self.playsplines[0].append(spline2)

            self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
        
        if (msg.cmd_right):
            pd_final_R = [msg.goal_right.x, msg.goal_right.y, msg.goal_right.z]
            pd_final_R = np.array(pd_final_R).reshape([3,1])

            # grab current position
            [pcur, _, _, _, _] = self.fbk.fkin()
            pcur_R = pcur[3:, :]
            pcur_R = np.array(pcur_R).reshape([3,1])

            # publish the goal points
            markermsg_R = create_pt_marker(pd_final_R[0,0], pd_final_R[1,0], pd_final_R[2,0], self.get_clock().now().to_msg(), color='r')
            self.pub_goalmarker_R.publish(markermsg_R)

            self.grip_effcmd[1] = GRIPPER_CLOSED_EFF

            # RIGHT SPLINES
            move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
            spline1 = Goto5(pcur_R, pd_final_R, move_time, space='task')
            spline2 = Stay(pd_final_R, space='task')
            if (self.playsplines[1] == None):
                self.playsplines[1] = [spline1]
            else:
                self.playsplines[1].append(spline1)
            self.playsplines[1].append(spline2)
            
            self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)

        # update the local planner state
        self.state = State.PLAY
        # tell the service that we suceeded
        response.success = True
        return response

    def cb_notecmd(self, msg, response):
        """
           Given a integer message with a MIDI note value, adds a spline that moves the tip from the current position to the position of the note on the keyboard
        """
        if self.kb_pos is None:
            response.success = False
            self.get_logger().info("No keyboard found!")
            return response
        
        # default grippers to open
        self.grip_effcmd = [float('NaN'), float('NaN')]

        # check the space of the message
        if (msg.space == 'task' or msg.space == 'keyboard'):
            # convert everything to task space
            # Keyboard position
            self.get_logger().info(f"Keyboard at {self.kb_pos.reshape([3])}")

            # Move to neutral position above keyboard.
            self.neutral_above_piano()

            # LEFT_NOTE
            if (msg.cmd_left):
                self.primitive_playnote(msg, left=True, space=msg.space)

            # RIGHT_NOTE
            if (msg.cmd_right):
                self.primitive_playnote(msg, left=False, space=msg.space)
    
        self.state = State.PLAY
        response.success = True
        return response


    def cb_pull_kb(self, _):
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

        center_ref_kb = np.array([*midi_helper.note_to_position("A3")]).reshape([3,1])
        center_ref_robot = self.kb_to_robot_frame(center_ref_kb, R_kb_world, delta)

        dist =  np.sqrt(center_ref_robot[0][0] ** 2 + center_ref_robot[1][0] ** 2)

        [pcur_L, _, _, _, _] = self.fbk.fkin()
        pcur_L = pcur_L[0:3, :]
        pcur_L = np.array(pcur_L).reshape([3,1])

        pulling = False

        if dist > .8:
            self.get_logger().info("No shot brother") 
        elif dist > .6:
            pulling = True

            pos_center_loop_kb = np.array([0.4, -.03, 0.]).reshape([3,1])
            pos_center_loop_world = self.kb_to_robot_frame(pos_center_loop_kb, R_kb_world, delta)

            target1 = np.array([pos_center_loop_world[0][0], pos_center_loop_world[1][0], .1]).reshape([3,1])
            target2 = pos_center_loop_world
            target3 = np.array([.6 * pos_center_loop_world[0][0], .6 * pos_center_loop_world[1][0], 0.]).reshape([3,1])

            move_time = 5
            spline1 = Goto5(pcur_L, target1, move_time, space='task')
            spline2 = Goto5(target1, target2, 1, space='task')
            spline3 = Goto5(target2, target3, move_time, space='task')
            if (self.playsplines[0] == None):
                self.playsplines[0] = [spline1]
            else:
                self.playsplines[0].append(spline1)
            self.playsplines[0].append(spline2)
            self.playsplines[0].append(spline3)

            self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
            self.grip_effcmd[0] = GRIPPER_CLOSED_EFF
        elif dist > 0.4:
            kb_angle = np.arctan2(self.kb_rot[1][0], self.kb_rot[0][0])
            angle_to_kb_center = -np.arctan2(center_ref_robot[1][0], -center_ref_robot[0][0])

            if kb_angle > np.deg2rad(30) + angle_to_kb_center:
                self.get_logger().info("Pulling right")

                if False: # TODO: something still wrong with this, be careful!
                    pulling = True

                    [pcur_R, _, _, _, _] = self.fbk.fkin()
                    pcur_R = pcur_L[3:6, :]
                    pcur_R = np.array(pcur_L).reshape([3,1])

                    pos_1_R = np.array([0.8, -.03, 0.03]).reshape([3,1])
                    pos_2_R = np.array([0.8, -.03, -0.0]).reshape([3,1])
                    target1 = self.kb_to_robot_frame(pos_1_R, R_kb_world, delta)
                    target2 = self.kb_to_robot_frame(pos_2_R, R_kb_world, delta)

                    loop_to_center = target2 - center_ref_robot
                    delta_angle = angle_to_kb_center - kb_angle
                    rot_matrix = np.array([
                            [np.cos(delta_angle), -np.sin(delta_angle), 0],
                            [np.sin(delta_angle), np.cos(delta_angle), 0],
                            [0., 0., 1.],
                        ]).reshape([3,3])
                    rotated_loop = rot_matrix @ loop_to_center
                    target3 = center_ref_robot + rotated_loop
                    print(pcur_R)
                    print(target1)
                    move_time = 5
                    spline1 = Goto5(pcur_R, target1, 1000, space='task')
                    spline2 = Goto5(target1, target2, 1, space='task')
                    spline3 = Goto5(target2, target3, move_time, space='task')
                    if (self.playsplines[1] == None):
                        self.playsplines[1] = [spline1]
                    else:
                        self.playsplines[1].append(spline1)
                    self.playsplines[1].append(spline2)
                    self.playsplines[1].append(spline3)

                    self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)
                    self.grip_effcmd[1] = GRIPPER_CLOSED_EFF

            elif kb_angle < np.deg2rad(-30) + angle_to_kb_center:
                self.get_logger().info("Pulling left")
                pulling = True

                pos_1_L = np.array([0.015, -.03, 0.03]).reshape([3,1])
                pos_2_L = np.array([0.015, -.03, -0.0]).reshape([3,1])
                target1 = self.kb_to_robot_frame(pos_1_L, R_kb_world, delta)
                target2 = self.kb_to_robot_frame(pos_2_L, R_kb_world, delta)

                loop_to_center = target2 - center_ref_robot
                delta_angle = angle_to_kb_center - kb_angle
                rot_matrix = np.array([
                        [np.cos(delta_angle), -np.sin(delta_angle), 0],
                        [np.sin(delta_angle), np.cos(delta_angle), 0],
                        [0., 0., 1.],
                    ]).reshape([3,3])
                rotated_loop = rot_matrix @ loop_to_center
                target3 = center_ref_robot + rotated_loop

                move_time = 5
                spline1 = Goto5(pcur_L, target1, move_time, space='task')
                spline2 = Goto5(target1, target2, 1, space='task')
                spline3 = Goto5(target2, target3, move_time, space='task')
                if (self.playsplines[0] == None):
                    self.playsplines[0] = [spline1]
                else:
                    self.playsplines[0].append(spline1)
                self.playsplines[0].append(spline2)
                self.playsplines[0].append(spline3)

                self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
                self.grip_effcmd[0] = GRIPPER_CLOSED_EFF

        if pulling:
            self.state = State.PLAY


    ## Helper functions
    # blocking function to grab a single feedback
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        self.fbkmsg = None
        def cb(fbkmsg):
            self.fbkmsg = fbkmsg
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, "/joint_states", cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)
        

        # Return the values.
        return self.fbkmsg

    def gravity(self):
        qm, _, _ = self.fbk.get_all_measured()

        t1_L = qm[7, 0]
        t2_L = qm[8, 0]
        t1_R = qm[3, 0]
        t2_R = qm[4, 0]

        def tau1_L(t1, t2):
            s1, c1, s2, c2, intr = 1.75577270e+00, 1.34368784e-02, 7.59615585e-01, 1.59857587e-03, 0.04459261 # new
            tau1 = s1 * np.sin(-t1) + c1 * np.cos(-t1) + s2 * np.sin(-t1 + t2) + c2 * np.cos(-t1 + t2) + intr
            return tau1
        def tau2_L(t1, t2):
            s2, c2, intr = -0.77427977, -0.03220803, 0.05437264 # new
            tau2 = s2 * np.sin(-t1 + t2) + c2 * np.cos(-t1 + t2) + intr
            return tau2
        def tau1_R(t1, t2):
            s1, c1, s2, c2, intr = 1.79597092, -0.00402647,  0.76979722, -0.02256849, 0.09761762
            tau1 = s1 * np.sin(-t1) + c1 * np.cos(-t1) + s2 * np.sin(-t1 + t2) + c2 * np.cos(-t1 + t2) + intr
            return tau1
        def tau2_R(t1, t2):
            s2, c2, intr = -0.69745138, 0.01699976, 0.01269389 # new
            tau2 = s2 * np.sin(-t1 + t2) + c2 * np.cos(-t1 + t2) + intr
            return tau2

        return [0., 0., tau1_L(t1_L, t2_L), tau2_L(t1_L, t2_L), 0., tau1_R(t1_R, t2_R), tau2_R(t1_R, t2_R)]
    
    def robot_to_kb_frame(self, p_robot, R_kb_world, delta):
        p_world = Rotz(np.pi).T @ p_robot + P_BASE_WORLD
        p_kb = R_kb_world.T @ (p_world - delta)
        return p_kb
    
    def kb_to_robot_frame(self, p_kb, R_kb_world, delta):
        p_world = R_kb_world @ (p_kb) + delta
        p_robot = Rotz(np.pi) @ (p_world - P_BASE_WORLD)
        return p_robot
    

    # PRIMITIVES
    def primitive_playnote(self, msg, left=True, space='task', first=False):
        # PERCEPTION
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

        if (space == 'task'):

            if (left):
                note = msg.note_left
                LRidx = 0
            else:
                note = msg.note_right
                LRidx = 1

            # extract location of note in keyboard frame
            nx, ny, nz = midi_helper.note_to_position(note)
            note_kb = np.array([nx, ny, nz]).reshape([3,1])

            # convert to world frame
            note_world = R_kb_world @ note_kb + delta

            ## Create the trajectory to move to the note
            # step 1: get starting robot position
            if (self.playsplines[LRidx]):
                # pull last position from last spline
                tend = self.playsplines[LRidx][-1].get_duration()
                p_start, _ = self.playsplines[LRidx][-1].evaluate(tend)
            else:
                # pull current position
                [p_start, _, _, _, _] = self.fbk.fkin()
                if (left):
                    p_start = p_start[0:3, :]
                    # initialize the clock
                    self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
                else:
                    p_start = p_start[3:, :]
                    # initialize the clock
                    self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)

            # step 2: get note position in robot frame
            pd_offsets = np.array([0., 0., -0.015]).reshape([3,1]) # m
            note_world = note_world + pd_offsets
            note_robot = Rotz(np.pi) @ (note_world - P_BASE_WORLD)

            # step 3: define intermediate points
            # point above the note
            pd_abovenote = note_robot + np.array([0.0, 0.0, 0.04]).reshape([3,1]) # m

            ## Generate and save splines
            # spline to above the note
            move_time = 1.25
            spline1_abovenote = Goto5(p_start, pd_abovenote, move_time, space='task')

            # spline to play the note
            move_time = 0.3 # TODO: compute based on desired force
            spline2_tonote = Goto5(pd_abovenote, note_robot, move_time, space='task')

            # hold at the note
            if (left):
                hold_time = msg.hold_time_left
            else:
                hold_time = msg.hold_time_right
            spline3_holdnote = Goto5(note_robot, note_robot, hold_time, space='task')

            # move back up above the note
            #move_time = distance_to_movetime(note_robot, pd_abovenote)
            move_time = 0.3
            spline4_afternote = Goto5(note_robot, pd_abovenote, move_time, space='task')

            # SAVE SPLINES
            if (self.playsplines[LRidx] == None):
                self.playsplines[LRidx] = [spline1_abovenote]
            else:
                self.playsplines[LRidx].append(spline1_abovenote)
            self.playsplines[LRidx].append(spline2_tonote)
            self.playsplines[LRidx].append(spline3_holdnote)
            self.playsplines[LRidx].append(spline4_afternote)
            
            # set the gripper to closed
            self.grip_effcmd[LRidx] = GRIPPER_CLOSED_EFF

        elif (space == 'keyboard'):
            if (left):
                note = msg.note_left
                LRidx = 0
            else:
                note = msg.note_right
                LRidx = 1

            # extract location of note in keyboard frame
            nx, ny, nz = midi_helper.note_to_position(note)
            note_kb = np.array([nx, ny, nz]).reshape([3,1])

            ## Create the trajectory to move to the note
            # step 1: get starting robot position in kb frame
            if (self.playsplines[LRidx]):
                # pull last position from last spline
                tend = self.playsplines[LRidx][-1].get_duration()
                p_start, _ = self.playsplines[LRidx][-1].evaluate(tend)

                if (self.playsplines[LRidx][-1].get_space() == 'task'):
                    self.get_logger().error("Tried to append a keyboard space spline to a task space spline.")
            else:
                # pull current position
                [p_start, _, _, _, _] = self.fbk.fkin()
                if (left):
                    p_start = p_start[0:3, :]
                    # initialize the clock
                    self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
                else:
                    p_start = p_start[3:, :]
                    # initialize the clock
                    self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)
                # convert to kb frame
                p_start = self.robot_to_kb_frame(p_start, R_kb_world, delta)

            # step 2: get note position in robot frame
            pd_offsets_kb = np.array([0., 0., -0.005]).reshape([3,1]) # m
            note_kb += pd_offsets_kb

            # step 3: define intermediate points
            # point above the note
            offset_kb = np.array([0.0, 0.0, 0.03]).reshape([3,1])
            pd_abovenote_kb = note_kb + offset_kb # m

            ## Generate and save splines
            # spline to above the note
            if (first):
                move_time = 1.25
            else:
                move_time = 0.4
            spline1_abovenote = Goto5(p_start, pd_abovenote_kb, move_time, space='keyboard')
            if (first):
                hold_time = 0.5
                spline1dot5_abovenotehold = Hold(pd_abovenote_kb, hold_time, space='keyboard')

            # spline to play the note
            move_time = 0.3 # TODO: compute based on desired force
            spline2_tonote = Goto5(pd_abovenote_kb, note_kb, move_time, space='keyboard')

            # hold at the note
            if (left):
                hold_time = msg.hold_time_left
            else:
                hold_time = msg.hold_time_right
            spline3_holdnote = Goto5(note_kb, note_kb, hold_time, space='keyboard')

            # move back up above the note
            # move_time = distance_to_movetime(note_kb, pd_abovenote_kb)
            move_time = 0.3
            spline4_afternote = Goto5(note_kb, pd_abovenote_kb, move_time, space='keyboard')

            # SAVE SPLINES
            if (self.playsplines[LRidx] == None):
                self.playsplines[LRidx] = [spline1_abovenote]
            else:
                self.playsplines[LRidx].append(spline1_abovenote)
            if (first):
                self.playsplines[LRidx].append(spline1dot5_abovenotehold)
            self.playsplines[LRidx].append(spline2_tonote)
            self.playsplines[LRidx].append(spline3_holdnote)
            self.playsplines[LRidx].append(spline4_afternote)
            
            # set the gripper to closed
            self.grip_effcmd[LRidx] = GRIPPER_CLOSED_EFF

    # TODO: CHECK
    def primitive_pauseabovenote(self, note, duration, left=True):
        # PERCEPTION
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

        # USES KEYBOARD SPACE
        if (left):
            LRidx = 0
        else:
            LRidx = 1

        # extract location of note in keyboard frame
        nx, ny, nz = midi_helper.note_to_position(note)
        note_kb = np.array([nx, ny, nz]).reshape([3,1])

        ## Create the trajectory to move to the note
        # step 1: get starting robot position in kb frame
        if (self.playsplines[LRidx]):
            # pull last position from last spline
            tend = self.playsplines[LRidx][-1].get_duration()
            p_start, _ = self.playsplines[LRidx][-1].evaluate(tend)

            if (self.playsplines[LRidx][-1].get_space() == 'task'):
                self.get_logger().error("Tried to append a keyboard space spline to a task space spline.")
        else:
            # pull current position
            [p_start, _, _, _, _] = self.fbk.fkin()
            if (left):
                p_start = p_start[0:3, :]
                # initialize the clock
                self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
            else:
                p_start = p_start[3:, :]
                # initialize the clock
                self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)
            # convert to kb frame
            p_start = self.robot_to_kb_frame(p_start, R_kb_world, delta)

        # step 2: get note position in robot frame
        pd_offsets_kb = np.array([0., 0., -0.01]).reshape([3,1]) # m
        note_kb += pd_offsets_kb

        # step 3: define intermediate points
        # point above the note
        offset_kb = np.array([0.0, 0.0, 0.04]).reshape([3,1])
        pd_abovenote_kb = note_kb + offset_kb # m

        ## Generate and save splines
        # spline to above the note
        move_time = duration
        spline1_abovenote = Goto5(p_start, pd_abovenote_kb, move_time, space='keyboard')

        # SAVE SPLINES
        if (self.playsplines[LRidx] == None):
            self.playsplines[LRidx] = [spline1_abovenote]
        else:
            self.playsplines[LRidx].append(spline1_abovenote)
        
        # set the gripper to closed
        self.grip_effcmd[LRidx] = GRIPPER_CLOSED_EFF

    def shrug(self):
        pass # TODO

    def neutral_above_piano(self):
        # get keyboard position
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

        # chill somewhere above the piano
        neutral_kb = [np.array([0.4, 0.0, 0.1]).reshape([3,1]), np.array([0.6, 0.0, 0.1]).reshape([3,1])]

        # spline from current position
        for LRidx in range(2):
            if (self.playsplines[LRidx]):
                # pull last position from last spline
                tend = self.playsplines[LRidx][-1].get_duration()
                p_start, _ = self.playsplines[LRidx][-1].evaluate(tend)

                if (self.playsplines[LRidx][-1].get_space() == 'task'):
                    self.get_logger().error("Tried to append a keyboard space spline to a task space spline.")
            else:
                # pull current position
                [p_start, _, _, _, _] = self.fbk.fkin()
                if (LRidx == 0):
                    p_start = p_start[0:3, :]
                    # initialize the clock
                    self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
                else:
                    p_start = p_start[3:, :]
                    # initialize the clock
                    self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)
                # convert to kb frame
                p_start = self.robot_to_kb_frame(p_start, R_kb_world, delta)

            # move_time = distance_to_movetime(p_start, neutral_kb[LRidx])
            move_time = 2.0
            spline1 = Goto5(p_start, neutral_kb[LRidx], move_time, space='keyboard')
            spline2 = Goto5(neutral_kb[LRidx], neutral_kb[LRidx], 2.0, space='keyboard')
            if (self.playsplines[LRidx] == None):
                self.playsplines[LRidx] = [spline1]
            else:
                self.playsplines[LRidx].append(spline1)
            self.playsplines[LRidx].append(spline2)


def distance_to_movetime(pstart, pend):
    T_MIN = 0.5 # s
    SPEED = 0.2 # m/s

    dist = np.linalg.norm(pstart - pend)
    time = max(T_MIN, dist / SPEED)
    return time

def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = LocalPlanner()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
