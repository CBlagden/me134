# local_planner.py
# ME 134 Team Penguinos
#
# Node to direct the robot's arms to a target end effector position.
# 
# SUBSCRIBES:
# /robot_state      me134_interfaces.msg/StateStamped
# /point_cmd        me134_interfaces.msg/PosCmdStamped
# /keyboarddetector/keyboard_point  geometry_msgs.msg/Point
# /note_cmd         me134_interfaces.srv/NoteCmdStamped
# 
# PUBLISHES:
# /joint_commands   sensor_msgs.msg/JointState
# /goal_marker      visualization_msgs.msg/Marker

import rclpy
from rclpy.node import Node

import numpy as np

from pianoman.state_machine.states import State
from pianoman.utils.rviz_helper import create_pt_marker
from pianoman.utils.KinematicChain import KinematicChain
from pianoman.utils.Segments import Goto5, Hold, Stay
import pianoman.utils.midi_helper as midi_helper
from pianoman.utils.TransformHelpers import Rotz
from pianoman.utils.StateClock import StateClock

from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import Point
from visualization_msgs.msg import Marker
from me134_interfaces.msg import StateStamped, PosCmdStamped
from me134_interfaces.srv import NoteCmdStamped

RATE = 100.0 # Hz
LAM = 10
P_BASE_WORLD = np.array([0.043, 0.593, 0.0]).reshape([3,1]) # m

class LocalPlanner(Node):
    P_HOVER = np.array([-0.5, 0.0, 0.1]).reshape([3,1]) # TODO: make not fixed (moves based on piano)

    def __init__(self):
        super().__init__('local_planner')

        ## Class variables
        # state variable
        self.state = State.default()
        # get initial joint position sensor reading
        [self.q, self.qdot, self.eff] = self.grabfbk()

        self.kb_pos = None

        # forward kinematics
        self.chain = KinematicChain("base_link", "tip_link")
        # general
        self.cursplines = []
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]

        # state clocks
        self.clocks = []
        self.play_clock = None


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
        self.sub_kb_pos = self.create_subscription(\
                Point, '/keyboarddetector/keyboard_point', self.cb_update_kb_pos, 10)

        ## Services
        self.srv_notecmd = self.create_service(\
                NoteCmdStamped, '/note_cmd', self.cb_notecmd)

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)


    # Main motor timer
    def cb_sendcmd(self):
        # Send commands based on the current state.
        if (self.state == State.PLAY):
            # Get current time (TODO: Proxy time that runs per each state)
            t = self.get_clock().now()
            dt = 1/RATE

            # Default to "floating" mode
            poscmd = [float("NaN"), float("NaN"), float("NaN")]
            velcmd = [float("NaN"), float("NaN"), float("NaN")]
            effcmd = self.gravity()

            # check if there is a spline to run
            if (self.cursplines):
                # check if completed
                curspline = self.cursplines[0]
                deltat = self.play_clock.t_since_start(t, rostime=True)

                if (curspline.completed(deltat)):
                    self.play_clock.restart(t, rostime=True)
                    self.cursplines.pop(0)
                else:
                    # joint space move
                    if (curspline.get_space() == 'joint'):
                        [qd, qd_dot] = curspline.evaluate(deltat)

                        poscmd = list(qd.reshape([3]))
                        velcmd = list(qd_dot.reshape([3]))

                    # task space move
                    elif (curspline.get_space() == 'task'):
                        [xd, xd_dot] = curspline.evaluate(deltat)
                        xd = xd.reshape([3,1])
                        xd_dot = xd_dot.reshape([3,1])

                        # compute forward kinematics
                        [xcurr, _, Jv, _] = self.chain.fkin(self.q)

                        # Augment Jv to remove singularities
                        GAM2 = 0.01
                        Jw_pinv = np.linalg.pinv(Jv.T @ Jv + GAM2*np.eye(3)) @ Jv.T

                        # get qdot with J qdot = xdot
                        ex = xd - xcurr
                        # qd_dot = np.linalg.pinv(Jv) @ (xd_dot + LAM * ex) # No anti-singularity weighting
                        qd_dot = Jw_pinv @ (xd_dot + LAM * ex)

                        # Add a secondary task to prevent the robot from running through the table
                        # TODO: THIS DOES NOT WORK
                        # Quadratic cost C(q1) = k q1^2 -> dC = 2k q1 = LAM_UP * q1
                        LAM_UP = 10
                        qdot_up = np.array([0.0, -LAM_UP * self.q[1,0], 0.0]).reshape([3,1])
                        qd_dot = qd_dot + (np.eye(3) - Jw_pinv @ np.linalg.pinv(Jw_pinv)) @ qdot_up
                        # qd_dot = qd_dot + (np.eye(3) - np.linalg.pinv(Jv) @ Jv) @ qdot_up   # No anti-singularity weighting

                        qd = self.q + qd_dot * dt

                        # save commands
                        poscmd = list(qd.reshape([3]))
                        velcmd = list(qd_dot.reshape([3]))

            # Publish!
            cmdmsg = JointState()
            cmdmsg.header.stamp = t.to_msg()
            cmdmsg.name = self.jointnames
            cmdmsg.position = poscmd
            cmdmsg.velocity = velcmd
            cmdmsg.effort = effcmd
            self.pub_jtcmd.publish(cmdmsg)


    ## Callback functions
    # callback for /robot_state
    def cb_updatestate(self, msg):
        self.state = State(msg.state_id)

    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint posiitons
        self.q = np.array(msg.position).reshape([3,1])
        self.qdot = np.array(msg.velocity).reshape([3,1])
        self.eff = np.array(msg.effort).reshape([3,1])

    # callback for /point_cmd
    def cb_pointcmd(self, msg: PosCmdStamped):
        #
        self.get_logger().info(f"Got point command: [{msg.goal.x}, {msg.goal.y}, {msg.goal.z}]")

        # reset spline list
        self.cursplines = []
        # Use this to initiate/reset splines between cartesian positions
        pd_final = [msg.goal.x, msg.goal.y, msg.goal.z]
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        pd_final = np.array(pd_final).reshape([3,1])

        move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
        self.cursplines.append(Goto5(pcur, pd_final, move_time, space='task'))
        # self.cursplines.append(Hold(pd_final, move_time, space='task'))
        self.cursplines.append(Stay(pd_final, space='task'))

        # initialize the clock
        self.play_clock = StateClock(self.get_clock().now(), rostime=True)

        # publish the goal point
        markermsg = create_pt_marker(msg.goal.x, msg.goal.y, msg.goal.z, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg)

    def cb_update_kb_pos(self, msg: Point):
        # technically z never changes but we save it just in case
        self.kb_pos = np.array([msg.x, msg.y, msg.z]).reshape([3,1])

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
        if (len(self.cursplines) > 0):
            tend = self.cursplines[-1].get_duration()
            p_start, _ = self.cursplines[-1].evaluate(tend)
        else:
            [p_start, _, _, _] = self.chain.fkin(self.q)
            # initialize the clock
            self.play_clock = StateClock(self.get_clock().now(), rostime=True)

        move_time = 0.6 # TODO: compute time based on distance
        self.cursplines.append(Goto5(p_start, pd_abovenote, move_time, space='task'))

        # now move to play the note
        move_time = 0.3 # TODO: compute based on desired force
        self.cursplines.append(Goto5(pd_abovenote, play_pd, move_time, space='task'))

        # finally, move back to a zeroed position by moving up and then across
        move_time = 0.5
        self.cursplines.append(Goto5(play_pd, pd_abovenote, move_time, space='task'))
        move_time = 0.6 # TODO: compute time based on distance
        # self.cursplines.append(Goto5(pd_abovenote, self.P_HOVER, move_time, space='task'))

        response.success = True
        return response

        # publish the goal point
        # markermsg = create_pt_marker(pd_final[0], pd_final[1], pd_final[2], self.get_clock().now().to_msg())
        # self.pub_goalmarker.publish(markermsg)


    ## Helper functions
    # blocking function to grab a single feedback
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        self.grabpos = None
        self.grabvel = None
        self.grabeff = None
        def cb(fbkmsg):
            self.grabpos = list(fbkmsg.position)
            self.grabvel = list(fbkmsg.velocity)
            self.grabeff = list(fbkmsg.effort)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, "/joint_states", cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return [self.grabpos, self.grabvel, self.grabeff]


    GRAV_A = -.1275
    GRAV_B = 0.
    GRAV_C = 2.5
    GRAV_D = 0.
    def gravity(self):
        _, t1, t2 = self.q
        tau1 = self.GRAV_A * np.sin(-t1 + t2) + \
               self.GRAV_B * np.cos(-t1 + t2) + \
               self.GRAV_C * np.sin(-t1) + \
               self.GRAV_D * np.cos(-t1)
        tau2 = self.GRAV_A * np.sin(-t1 + t2) + \
               self.GRAV_B * np.cos(-t1 + t2)
        return [0., float(tau1), float(tau2)]


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
