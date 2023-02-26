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

import numpy as np
import csv

from pianoman.state_machine.states import State
from pianoman.utils.rviz_helper import create_pt_marker
from pianoman.utils.Segments import Goto5, Hold, Stay
import pianoman.utils.midi_helper as midi_helper
from pianoman.utils.TransformHelpers import Rotz, R_from_quat
from pianoman.utils.StateClock import StateClock
from pianoman.utils.gravitycomp import get_data

from pianoman.local_planner.jointstate_helper import JointStateHelper

from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import Pose
from visualization_msgs.msg import Marker
from me134_interfaces.msg import StateStamped, PosCmdStamped
from me134_interfaces.srv import NoteCmdStamped

# TODO: remove, make JointCmdStamped
from std_msgs.msg import Float64MultiArray

RATE = 100.0 # Hz
LAM = 3
P_BASE_WORLD = np.array([0.043, 0.593, 0.0]).reshape([3,1]) # m

L_IDX = [0, 6, 7, 8]
R_IDX = [0, 2, 3, 4]
GRIP_IDX = [1, 5]
JOINT_NAMES = ["base_joint", \
                "R_gripper_joint", "R_pan_joint", "R_lower_joint", "R_upper_joint", \
                "L_gripper_joint", "L_pan_joint", "L_lower_joint", "L_upper_joint"]

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        ## Class variables
        # state variable
        self.state = State.default()
        # get initial joint position sensor reading
        self.fbk = JointStateHelper(JOINT_NAMES, L_IDX, R_IDX, GRIP_IDX)
        self.fbk.update_measurements(self.grabfbk())

        self.kb_pos = None
        self.kb_rot = None

        # general
        self.playsplines = []
        self.grip_poscmd = [float("NaN"), float("NaN")]

        # state clocks
        self.clocks = []
        self.play_clock = None
        # time variables
        self.tstart = None

        ## Publishers
        self.pub_jtcmd = self.create_publisher(\
                JointState, '/joint_commands', 10)
        self.pub_goalmarker = self.create_publisher(\
                Marker, '/goal_marker', 10)

        ## Subscribers
        self.sub_jtstate = self.create_subscription(\
                JointState, '/joint_states', self.cb_jtstate, 10)
        self.sub_pointcmd = self.create_subscription(\
                PosCmdStamped, '/point_cmd', self.cb_pointcmd, 10)
        self.sub_jointcmd = self.create_subscription(\
                Float64MultiArray, '/joint_cmd', self.cb_jointcmd, 10)
        self.sub_kb_pos = self.create_subscription(\
                Pose, '/keyboarddetector/keyboard_point', self.cb_update_kb_pos, 10)

        ## Services
        self.srv_notecmd = self.create_service(\
                NoteCmdStamped, '/note_cmd', self.cb_notecmd)

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)

        self.grav_data = get_data()
        q1, q2, dur = self.grav_data.pop(0)
        qd_final = np.array([0, 0, q1, q2, 0, 0, 0]).reshape([7,1])
        qcur = self.fbk.get_joints_measured()[0]
        qcur = np.array(qcur).reshape([7,1])
        self.playsplines.append(Goto5(qcur, qd_final, dur, space='joint'))
        self.tstart = self.get_clock().now()
        
    # Main motor timer
    def cb_sendcmd(self):
        # Send commands based on the current state
        if (self.state == State.PLAY):
            # get current time
            t = self.get_clock().now()
            dt = 1/RATE

            # Default to floating mode
            poscmd = 7 * [float("NaN")]
            velcmd = 7 * [float("NaN")]
            effcmd = 7 * [float("NaN")]

            # check if there is a spline to run
            if (self.playsplines):
                curspline = self.playsplines[0]
                # deltat = self.play_clock.t_since_start(t, rostime=True)
                deltat = (t - self.tstart).nanoseconds / 1000000000.

                if (curspline.completed(deltat)):
                    # Remove the spline if completed
                    # self.play_clock.restart(t, rostime=True)
                    self.tstart = t
                    self.playsplines.pop(0)
                    if self.grav_data:
                        qm, _, _ = self.fbk.get_all_measured()

                        t1_L = qm[7, 0]
                        t2_L = qm[8, 0]
                        t1_R = qm[3, 0]
                        t2_R = qm[4, 0]

                        [_, _, eff1, eff2, _, _, _] = self.fbk.get_joints_measured()[2].flatten()

                        with open('grav_data.csv', 'a') as datafile:
                            writer = csv.writer(datafile)
                            writer.writerow([np.sin(-t1_L), np.sin(-t1_L + t2_L), eff1, eff2])

                        q1, q2, dur = self.grav_data.pop(0)
                        qd_final = np.array([0, 0, q1, q2, 0, 0, 0]).reshape([7,1])
                        qcur = self.fbk.get_joints_measured()[0]
                        qcur = np.array(qcur).reshape([7,1])
                        self.playsplines.append(Goto5(qcur, qd_final, dur, space='joint'))
                        self.get_logger().info("Going to %.04f, %.04f" % (q1, q2))
                else:
                    # joint space
                    if (curspline.get_space() == 'joint'):
                        [qd, qd_dot] = curspline.evaluate(deltat)

                        poscmd = list(qd.reshape([7]))
                        velcmd = list(qd_dot.reshape([7]))

                    # Just move the left arm (TODO: allow arm selection)
                    elif (curspline.get_space() == 'task'):
                        # pull out end effector coordinates
                        [xd, xd_dot] = curspline.evaluate(deltat)
                        xd = xd.reshape([3,1])
                        xd_dot = xd_dot.reshape([3,1])

                        # run through forward kinematics
                        [xcurr, _, _, Jv, _] = self.fbk.fkin()
                        # remove the last three columns of Jv (they correspond to the right arm)
                        Jv = Jv[0:3, 0:4]
                        xcurr = xcurr[0:3,:]  # pull out left arm positions

                        # get qdot with J qdot = xdot
                        ex = xd - xcurr
                        qd_dot = np.linalg.pinv(Jv) @ (xd_dot + LAM * ex)


                        # get desired q from Euler integration with qdot
                        q, _, _ = self.fbk.get_joints_measured()
                        # only pull out left
                        q = q[L_IDX, :]
                        qd = q + qd_dot * dt

                        # save commands
                        poscmd = list(qd.reshape([4])) + 3 * [float("NaN")]
                        velcmd = list(qd_dot.reshape([4])) + 3 * [float("NaN")]
                    
                    # left and right task space move
                    elif (curspline.get_space() == '2task'):
                        # pull out end effector coordinates
                        [xd, xd_dot] = curspline.evaluate(deltat)
                        xd = xd.reshape([6,1])
                        xd_dot = xd_dot.reshape([6,1])

                        # run through forward kinematics
                        [xcurr, _, _, Jv, _] = self.fbk.fkin()

                        # get qdot with J qdot = xdot
                        ex = xd - curr
                        qd_dot = np.linalg.pinv(Jv) @ (xd_dot + LAM * ex)


                        # get desired q from Euler integration with qdot
                        q, _, _ = self.fbk.get_joints_measured()
                        qd = q + qd_dot * dt

                        # save commands
                        poscmd = list(qd.reshape([7]))
                        velcmd = list(qd_dot.reshape([7]))

            # Publish!
            cmdmsg = self.fbk.to_msg(t, poscmd, velcmd, effcmd, self.grip_poscmd)
            self.pub_jtcmd.publish(cmdmsg)


    ## Callback functions
    # callback for /robot_state
    def cb_updatestate(self, msg):
        self.state = State(msg.state_id)

    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint posiitons
        self.fbk.update_measurements(msg)

        [p, _, _, _, _] = self.fbk.fkin()

    def cb_update_kb_pos(self, msg: Pose):
        # technically z never changes but we save it just in case
        self.kb_pos = np.array([msg.position.x, msg.position.y, msg.position.z]).reshape([3,1])
        self.kb_rot = R_from_quat(np.array([msg.orientation.x,
                                            msg.orientation.y,
                                            msg.orientation.z,
                                            msg.orientation.w])).reshape((3, 3))

    # callback for /joint_cmd
    def cb_jointcmd(self, msg: Float64MultiArray):
        #
        self.get_logger().info("Got joint command: {}".format(msg.data))

        # reset spline list
        self.playsplines = []
        # Use this to initiate/reset splines between cartesian positions
        qd_final = list(msg.data)
        qcur = self.fbk.get_joints_measured()[0]

        print("qcur {}".format(qcur))
        qd_final = np.array(qd_final).reshape([7,1])
        qcur = np.array(qcur).reshape([7,1])

        move_time = 20
        self.playsplines.append(Goto5(qcur, qd_final, move_time, space='joint'))
        # self.cursplines.append(Hold(pd_final, move_time, space='task'))
        self.playsplines.append(Stay(qd_final, space='joint'))
        self.tstart = self.get_clock().now()

    # callback for /point_cmd
    def cb_pointcmd(self, msg: PosCmdStamped):
        #
        self.get_logger().info(f"Got point command: [{msg.goal.x}, {msg.goal.y}, {msg.goal.z}]")

        # reset spline list
        self.playsplines = []
        # Use this to initiate/reset splines between cartesian positions
        pd_final = [msg.goal.x, msg.goal.y, msg.goal.z]
        [pcur, _, _, _, _] = self.fbk.fkin()
        pcur = pcur[0:3, :]
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        print("pcur {}".format(pcur))
        pd_final = np.array(pd_final).reshape([3,1])

        move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
        self.playsplines.append(Goto5(pcur, pd_final, move_time, space='task'))
        # self.cursplines.append(Hold(pd_final, move_time, space='task'))
        self.playsplines.append(Stay(pd_final, space='task'))
        self.tstart = self.get_clock().now()

        # publish the goal point
        markermsg = create_pt_marker(msg.goal.x, msg.goal.y, msg.goal.z, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg)

    def cb_notecmd(self, msg, response):
        """
           Given a integer message with a MIDI note value, adds a spline that moves the tip from the current position to the position of the note on the keyboard
        """
        note = msg.note

        # extract location of note in keyboard frame
        nx, ny, nz = midi_helper.note_to_position(note)
        note_kb = np.array([nx, ny, nz]).reshape([3,1])

        # Convert to world frame
        R_kb_world = Rotz(-np.pi/2) # TODO: update this based on perception
        note_world = R_kb_world @ note_kb + self.kb_pos

        # print to logger
        self.get_logger().info(f"Added {note} at [{note_world[0]}, {note_world[1]}, {note_world[2]}]")

        # Create the trajectory to move to the note
        pd_offsets = np.array([0.04, -0.07, -0.02]).reshape([3,1]) # m
        # convert to robot base frame coordinates  and add offsets
        play_pd = Rotz(np.pi) @ ((note_world + pd_offsets) - P_BASE_WORLD)

        ## Add the note to the trajectory!
        # first move to a point above the note
        pd_abovenote = play_pd + np.array([0.0, 0.0, 0.04]).reshape([3,1]) # m
        if (len(self.playsplines) > 0):
            tend = self.playsplines[-1].get_duration()
            p_start, _ = self.playsplines[-1].evaluate(tend)
        else:
            [p_start, _, _, _, _] = self.fbk.fkin()
            p_start = p_start[0:3, :]
            # initialize the clock
            self.play_clock = StateClock(self.get_clock().now(), rostime=True)

        move_time = 0.6 # TODO: compute time based on distance
        p_start2 = np.vstack([p_start])
        self.playsplines.append(Goto5(p_start, pd_abovenote, move_time, space='task'))

        # now move to play the note
        move_time = 0.3 # TODO: compute based on desired force
        self.playsplines.append(Goto5(pd_abovenote, play_pd, move_time, space='task'))

        # finally, move back to a zeroed position by moving up and then across
        move_time = 0.5
        self.playsplines.append(Goto5(play_pd, pd_abovenote, move_time, space='task'))
        move_time = 0.6 # TODO: compute time based on distance
        # self.playsplines.append(Goto5(pd_abovenote, self.P_HOVER, move_time, space='task'))

        response.success = True
        return response

        # publish the goal point
        # markermsg = create_pt_marker(pd_final[0], pd_final[1], pd_final[2], self.get_clock().now().to_msg())
        # self.pub_goalmarker.publish(markermsg)


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


    GRAV_A = -0.75
    GRAV_C = 3

    def gravity(self):
        qm, _, _ = self.fbk.get_all_measured()

        t1_L = qm[7, 0]
        t2_L = qm[8, 0]
        t1_R = qm[3, 0]
        t2_R = qm[4, 0]

        def tau1(t1, t2):
            tau1 = .75 * self.GRAV_A * np.sin(-t1 + t2) + \
                self.GRAV_C * np.sin(-t1)
            return tau1
        def tau2(t1, t2):
            tau2 = self.GRAV_A * np.sin(-t1 + t2)
            return tau2
        
        # return 7 * [0.]
        # return [0., 0., tau1(t1_L, t2_L), tau2(t1_L, t2_L), 0., 0., 0.]
        # return [0., 0., 0., 0., 0., tau1(t1_R, t2_R), tau2(t1_R, t2_R)]
        return [0., 0., tau1(t1_L, t2_L), tau2(t1_L, t2_L), 0., tau1(t1_R, t2_R), tau2(t1_R, t2_R)]


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
