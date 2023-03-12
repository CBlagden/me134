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

from std_msgs.msg import Empty
from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import Pose
from visualization_msgs.msg import Marker
from me134_interfaces.msg import PoseTwoStamped
from me134_interfaces.srv import NoteCmdStamped, PosCmdStamped

# TODO: remove, make JointCmdStamped
from std_msgs.msg import Float64MultiArray

RATE = 100.0 # Hz
LAM = 18

# P_BASE_WORLD = np.array([-0.018, 0.69, 0.0]).reshape([3, 1]) # m
P_BASE_WORLD = np.array([-0.018 + 0.03, 0.69 + 0.01, 0.0]).reshape([3, 1]) # m

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
        self.position_cmd = np.array(6 * [0.]).reshape([6,1])
        # get initial joint position sensor reading
        self.fbk = JointStateHelper(JOINT_NAMES, L_IDX, R_IDX, GRIP_IDX)
        self.fbk.update_measurements(self.grabfbk())

        self.kb_pos = None
        self.kb_rot = None

        q0, _, _ = self.fbk.get_joints_measured()
        self.qd = q0

        # general
        self.playsplines = []
        self.grip_poscmd = [float("NaN"), float("NaN")]

        # state clocks
        self.cur_clock = None
        self.play_clock = None
        self.hit_clock = None
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
            self.grip_poscmd = [float("NaN"), float("NaN")]

            q, _, _ = self.fbk.get_joints_measured()
            self.qd = q
            self.playsplines = []

            cmdmsg = self.fbk.to_msg(self.get_clock().now(), poscmd, velcmd, effcmd, self.grip_poscmd)
            self.pub_jtcmd.publish(cmdmsg)


        if (self.state == State.PLAY):
            # get current time
            t = self.get_clock().now()
            dt = 1/RATE

            # Default to floating mode
            poscmd = 7 * [float("NaN")]
            velcmd = 7 * [float("NaN")]
            effcmd = self.gravity()

            # check if there is a spline to run
            if (self.playsplines):
                curspline = self.playsplines[0]
                deltat = self.play_clock.t_since_start(t, rostime=True)
                # deltat = (t - self.tstart).nanoseconds / 1000000000.

                if (curspline.completed(deltat)):
                    # Remove the spline if completed
                    self.play_clock.restart(t, rostime=True)
                    # self.tstart = t
                    self.playsplines.pop(0)

                else:
                    # For task space moves
                    if (curspline.get_space() == 'task' or curspline.get_space() == '2task'):
                        # start with forward kinematics
                        [xcurr, _, _, Jv, _] = self.fbk.fkin(self.qd)

                        # perform space-specific parts
                        if (curspline.get_space() == 'task'):
                            # left arm only
                            num_arms = 1
                            # remove the last three columns of Jv (they correspond to the right arm)
                            Jv = Jv[0:3, 0:4]
                            xcurr = xcurr[0:3,:]  # pull out left arm positions

                        elif (curspline.get_space() == '2task'):
                            # both arms
                            num_arms = 2

                            # both arms does not need to change Jv or xcurr

                        # Evaluate the splines to get desired ee coordinates
                        [xd, xd_dot] = curspline.evaluate(deltat)
                        # Save xd for publishing pose error later
                        self.position_cmd = xd.copy().reshape([3 * num_arms,1]) # save xd for publishing 
                        # Reformat xd and xd_dot for kinematics
                        xd = xd.reshape([3 * num_arms,1])
                        xd_dot = xd_dot.reshape([3 * num_arms,1])

                        qd = self.qd[0:(1 + 3 * num_arms)]

                        # Weighted pseudoinverse for singularity prevention
                        GAM2 = 0.01
                        Jv_pinv = np.linalg.pinv(Jv.T @ Jv + GAM2*np.eye(1 + 3*num_arms)) @ Jv.T
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

                        l_base = 0.3
                        l_pan = 0.1
                        l_lower = 0.3
                        l_upper = 0.3

                        if num_arms == 1:
                            lambda_s = np.array([l_base, l_pan, l_lower, l_upper]).reshape((4, 1))
                            # we only have a left arm
                            q_sec_goal[2, 0] = 0.9 # left lower joint
                            q_sec_goal[3, 0] = -1.2 # left upper joint
                            
                            qd_sec = lambda_s * (q_sec_goal - qd)

                            qd_sec[1, 0] = 0.0 # left pan joint stays free

                        else:
                            # we have two arms, yay us!  
                            lambda_s = np.array([l_base, l_pan, l_lower, l_upper, l_pan, l_lower, l_upper]).reshape((7, 1))
                            q_sec_goal[2, 0] = 0.9 # right lower joint
                            q_sec_goal[3, 0] = -1.2 # right upper joint
                            q_sec_goal[5, 0] = -0.9 # left lower joint
                            q_sec_goal[6, 0] = 1.2 # left upper joint
                            
                            qd_sec = lambda_s * (q_sec_goal - qd)
                            qd_sec[1, 0] = 0.0 # right pan joint stays free
                            qd_sec[4, 0] = 0.0 # left pan joint stays free
                        

                        # Inverse kinematics!
                        qd_dot = Jv_pinv @ (xd_dot + LAM * ex)

                        # Map secondary tasks to the null space
                        qd_dot += (np.eye(1 + 3 * num_arms) - Jv_pinv @ np.linalg.pinv(Jv_pinv)) @ qd_sec # COMMENT OUT TO REMOVE SECONDARY TASKS

                        # get desired q from Euler integration with qdot
                        qd += qd_dot * dt
                        self.qd[0:(1 + 3 * num_arms)] = qd # TODO: TEST/CHECK

                        # save commands
                        if (curspline.get_space() == 'task'):
                            poscmd = list(self.qd[:4, 0].reshape([4])) + 3 * [float("NaN")]
                            velcmd = list(qd_dot.reshape([4])) + 3 * [float("NaN")]
                        elif (curspline.get_space() == '2task'):
                            poscmd = list(self.qd.reshape([7]))
                            velcmd = list(qd_dot.reshape([7]))
            else:
                # If no spline to run, return to floating mode
                self.state = State.FLOAT
                

            # Publish!
            cmdmsg = self.fbk.to_msg(t, poscmd, velcmd, effcmd, self.grip_poscmd)
            self.pub_jtcmd.publish(cmdmsg)

            # save effort command and stamp
            self.eff_cmd = np.array(cmdmsg.effort).reshape([9,1])
            self.eff_cmd_stamp = cmdmsg.header.stamp.sec + 1e-9 * cmdmsg.header.stamp.nanosec

    def cb_songcmd(self, msg, response):
        """
            Given a message with a path to a midi file, parses the file
            and plays the accompanying song
        """
        filename = msg.data
        left_traj, right_traj = midi_helper.note_trajectories(filename)
        


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
        self.playsplines = []

        if (msg.left_only):
            # Command only the left
            space = 'task'
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
            
            self.grip_poscmd = [-0.7, float("NaN")]
        else:
            # command both left and right
            space = '2task'
            pd_final = [msg.goal_left.x, msg.goal_left.y, msg.goal_left.z, \
                        msg.goal_right.x, msg.goal_right.y, msg.goal_right.z]
            pd_final = np.array(pd_final).reshape([6,1])

            # grab current position
            [pcur, _, _, _, _] = self.fbk.fkin()
            pcur = np.array(pcur).reshape([6,1])

            # publish the goal points
            markermsg_L = create_pt_marker(pd_final[0,0], pd_final[1,0], pd_final[2,0], self.get_clock().now().to_msg())
            markermsg_R = create_pt_marker(pd_final[3,0], pd_final[4,0], pd_final[5,0], self.get_clock().now().to_msg(), color='r')
            self.pub_goalmarker_L.publish(markermsg_L)
            self.pub_goalmarker_R.publish(markermsg_R)

            self.grip_poscmd = 2 * [-0.7]

            # # WARNING: DO NOT DO THIS FOR NOTES
            # q, _, _ = self.fbk.get_joints_measured()
            # self.qd = q

        # create the splines
        # print("pcur {}".format(pcur))
        move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
        # move_time = distance_to_movetime(pcur, pd_final)
        self.playsplines.append(Goto5(pcur, pd_final, move_time, space=space))
        # self.cursplines.append(Hold(pd_final, move_time, space=space))
        self.playsplines.append(Stay(pd_final, space=space))
        # self.tstart = self.get_clock().now()
        self.play_clock = StateClock(self.get_clock().now(), rostime=True)



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

        # Keyboard position stuff (ONLY uses perception)
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

        self.get_logger().info(f"Keyboard at {self.kb_pos}")

        if (len(self.playsplines) > 0):
            tend = self.playsplines[-1].get_duration()
            p_start, _ = self.playsplines[-1].evaluate(tend)
        else:
            [p_start, _, _, _, _] = self.fbk.fkin()
            # initialize the clock
            self.play_clock = StateClock(self.get_clock().now(), rostime=True)

        # Handle the LEFT_NOTE
        if (msg.left_only):
            space = 'task'
            note_L = msg.note_left
            # extract location of note in keyboard frame
            nx, ny, nz = midi_helper.note_to_position(note_L)

            note_kb = np.array([nx, ny, nz]).reshape([3,1])

            note_world = R_kb_world @ note_kb + delta

            # print to logger
            self.get_logger().info(f"Added {note_L} at [{note_world[0]}, {note_world[1]}, {note_world[2]}]")

            # Create the trajectory to move to the note
            pd_offsets = np.array([0., 0., -0.02]).reshape([3,1]) # m
            # convert to robot base frame coordinates  and add offsets
            play_pd = Rotz(np.pi) @ ((note_world + pd_offsets) - P_BASE_WORLD)
            self.get_logger().info(f"Goal point: {play_pd.reshape([3])}")

            ## Add the note to the trajectory!
            # first move to a point above the note
            pd_abovenote = play_pd + np.array([0.0, 0.0, 0.04]).reshape([3,1]) # m

            # publish the goal point
            # markermsg = create_pt_marker(*list(play_pd.flatten()), self.get_clock().now().to_msg())
            # self.pub_goalmarker_L.publish(markermsg)

            self.grip_poscmd = [-0.7, float("NaN")]

            # q, _, _ = self.fbk.get_joints_measured()
            # self.qd = q[0:4, :]

            p_start = p_start[0:3, :]
            move_time_initial = distance_to_movetime(p_start, pd_abovenote)

        else:
            space = '2task'
            note_L = msg.note_left
            note_R = msg.note_right
            # extract location of note in keyboard frame
            nLx, nLy, nLz = midi_helper.note_to_position(note_L)
            nRx, nRy, nRz = midi_helper.note_to_position(note_R)
            note_L_kb = np.array([nLx, nLy, nLz]).reshape([3,1])
            note_R_kb = np.array([nRx, nRy, nRz]).reshape([3,1])

            note_L_world = R_kb_world @ note_L_kb + delta
            note_R_world = R_kb_world @ note_R_kb + delta

            # print to logger
            self.get_logger().info(f"Added {note_L} at [{note_L_world[0]}, {note_L_world[1]}, {note_L_world[2]}]")

            # Create the trajectory to move to the note
            # pd_offsets = np.array([0.0, 0., -0.05]).reshape([3,1]) # m
            pd_offsets = np.array([0., 0., -0.02]).reshape([3,1]) # m
            # convert to robot base frame coordinates  and add offsets
            play_pd_L = Rotz(np.pi) @ ((note_L_world + pd_offsets) - P_BASE_WORLD)
            play_pd_R = Rotz(np.pi) @ ((note_R_world + pd_offsets) - P_BASE_WORLD)
            self.get_logger().info(f"Left goal point: {play_pd_L}")
            self.get_logger().info(f"Right goal point: {play_pd_R}")

            ## Add the note to the trajectory!
            # first move to a point above the note
            pd_L_abovenote = play_pd_L + np.array([0.0, 0.0, 0.04]).reshape([3,1]) # m
            pd_R_abovenote = play_pd_R + np.array([0.0, 0.0, 0.04]).reshape([3,1]) # m

            # combine!
            play_pd = np.vstack([play_pd_L, play_pd_R])
            pd_abovenote = np.vstack([pd_L_abovenote, pd_R_abovenote])

            # publish the goal point
            # markermsg = create_pt_marker(*list(play_pd.flatten()), self.get_clock().now().to_msg())
            # self.pub_goalmarker_L.publish(markermsg)
            
            self.grip_poscmd = [-0.7, -0.7]
            move_time_initial = distance_to_movetime(p_start, pd_abovenote)

            # q, _, _ = self.fbk.get_joints_measured()
            # self.qd = q


        self.playsplines.append(Goto5(p_start, pd_abovenote, 1 * move_time_initial, space=space))

        # now move to play the note
        move_time = 0.3 # TODO: compute based on desired force
        self.playsplines.append(Goto5(pd_abovenote, play_pd, move_time, space=space))

        hold_time = msg.hold_time_left
        self.playsplines.append(Goto5(play_pd, play_pd, hold_time, space=space))

        # finally, move back to a zeroed position by moving up and then across
        move_time = 0.6 # TODO: compute time based on distance
        self.playsplines.append(Goto5(play_pd, pd_abovenote, move_time, space=space))
        # self.playsplines.append(Goto5(pd_abovenote, self.P_HOVER, move_time, space='task'))
    
        self.state = State.PLAY
        response.success = True
        return response


    def cb_pull_kb(self, _):
        nx, ny, nz = midi_helper.note_to_position('C2')
        note_kb = np.array([nx, ny, nz]).reshape([3,1])

        delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))
        delta += np.array([0, 0, 0]).reshape([3,1])
        R_kb_world = Rotz(-np.pi/2) @ self.kb_rot # TODO: update this based on perception
        target = R_kb_world @ note_kb + delta

        self.get_logger().info('Target: %f %f %f' % (float(target[0][0]), float(target[1][0]), float(target[2][0])))
        markermsg = create_pt_marker(*list(target.flatten()), '23rf4')
        print("TARGET:", target)
        self.pub_goalmarker_L.publish(markermsg)

        target[0] *= .8
        target[1] *= .8
        
        self.get_logger().info('Target: %f %f %f' % (float(target[0][0]), float(target[1][0]), float(target[2][0])))
        markermsg = create_pt_marker(*list(target.flatten()), self.get_clock().now().to_msg())
        self.pub_goalmarker_L.publish(markermsg)
        print("TARGET:", target)


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


def distance_to_movetime(pstart, pend):
    T_MIN = 0.5 # s
    SPEED = 0.2 # m/s

    dist = np.linalg.norm(pstart - pend)
    time = max(0.2, dist / SPEED)
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
