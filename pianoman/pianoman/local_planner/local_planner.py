# local_planner.py
# ME 134 Team Penguinos
#
# Node to direct the robot's arms to a target end effector position.
# 
# SUBSCRIBES:
# /robot_state      me134_interfaces.msg/StateStamped
# /point_cmd        me134_interfaces.msg/PosCmdStamped
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

from sensor_msgs.msg    import JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, String
from me134_interfaces.msg import StateStamped, PosCmdStamped

RATE = 100.0 # Hz
LAM = 10

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        ## Class variables
        # state variable
        self.state = State.default()
        # get initial joint position sensor reading
        [self.q, self.qdot] = self.grabfbk()

        self.kb_pos = None

        # forward kinematics
        self.chain = KinematicChain("base_link", "tip_link")
        # time variables
        self.tstart = None
        # general
        self.cursplines = []
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]


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
        self.sub_notecmd = self.create_subscription(\
                String, '/note_cmd', self.cb_notecmd, 10)

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
                deltat = (t - self.tstart).nanoseconds / 1000000000.
                if (curspline.completed(deltat)):
                    self.tstart = t
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

    # callback for /point_cmd
    def cb_pointcmd(self, msg):
        #
        self.get_logger().info("Got point command.")

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
        self.tstart = self.get_clock().now()

        # publish the goal point
        markermsg = create_pt_marker(msg.goal.x, msg.goal.y, msg.goal.z, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg)

    def cb_update_kb_pos(self, msg: Point):
        # technically z never changes but we save it just in case
        self.kb_pos = [msg.x, msg.y, msg.z]

    def cb_notecmd(self, msg: String):
        """
           Given a integer message with a MIDI note value, adds a spline that moves the tip from the current position to the position of the note on the keyboard
        """
        note = int(msg.data)
        kb_goal_x, kb_goal_y = midi_helper.note_to_position(note)
        kb_pos_x, kb_pos_y, _ = self.kb_pos
        goal_x = kb_goal_x + kb_pos_x
        goal_y = kb_goal_y + kb_pos_y
        pd_final = [goal_x, goal_y, 30] # idk what the units are

        # the rest of this is identical to self.cb_pointcmd
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        pd_final = np.array(pd_final).reshape([3,1])

        move_time = msg.move_time.sec + msg.move_time.nanosec * 1e-9
        self.cursplines.append(Goto5(pcur, pd_final, move_time, space='task'))
        # self.cursplines.append(Hold(pd_final, move_time, space='task'))
        self.cursplines.append(Stay(pd_final, space='task'))
        self.tstart = self.get_clock().now()

        # publish the goal point
        markermsg = create_pt_marker(msg.goal.x, msg.goal.y, msg.goal.z, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg)


    ## Helper functions
    # blocking function to grab a single feedback
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        self.grabpos = None
        self.grabvel = None
        def cb(fbkmsg):
            self.grabpos = list(fbkmsg.position)
            self.grabvel = list(fbkmsg.velocity)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, "/joint_states", cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return [self.grabpos, self.grabvel]


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
