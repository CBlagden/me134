# GotoNode
# Moves the robot along a cartesian or joint space spline.
#
# ros2 topic pub --once /point_cmd geometry_msgs/Point "{x: -0.2, y: 0.1, z: 0.5}"
# 
# Publish:
#   /joint_commands     sensor_msgs/JointState
# 
# Subscribe:
#   /joint_states       sensor_msgs/JointState
#   /point_cmd           geometry_msgs/Point
#
# ME 134 LOS PENGUINOS

import rclpy
from rclpy.node import Node
import rclpy.time

import numpy as np
import time

from threedof_demo.KinematicChain import KinematicChain
from threedof_demo.Segments import Goto5
from threedof_demo.analytic_solver import get_sols
from threedof_demo.TransformHelpers import *

from sensor_msgs.msg    import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from std_msgs.msg import Empty

RATE = 20.0
LAM = 10

TIME = 7

L1 = 0.508
L2 = 0.316

class GotoNode(Node):
    def __init__(self):
        super().__init__('goto_node')

        ## Class variables
        # get initial joint position sensor reading
        [self.q, self.qdot] = self.grabfbk()
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
                Point, '/point_cmd', self.cb_pointcmd, 10)
        self.sub_flipcmd = self.create_subscription(Empty, '/flip', self.cb_flip, 1)

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)

    def cb_flip(self, _):
        self.get_logger().info("Flipping...")
        new_q1 = np.array([self.q[0][0] - np.pi, self.q[1][0], self.q[2][0]]).reshape((3, 1))
        self.cursplines.append(Goto5(self.q, new_q1, TIME, space='joint'))
        new_q2 = np.array([new_q1[0][0], np.pi - new_q1[1][0], -new_q1[2][0]]).reshape((3, 1))
        self.cursplines.append(Goto5(new_q1, new_q2, TIME, space='joint'))

    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint posiitons
        self.q = np.array(msg.position).reshape([3,1])
        self.qdot = np.array(msg.velocity).reshape([3,1])

    
    # callback for /point_cmd
    def cb_pointcmd(self, msg):
        # Use this to initiate/reset splines between cartesian positions
        pd_final = [msg.x, msg.y, msg.z]
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        pd_final = np.array(pd_final).reshape([3,1])
        move_time = TIME #s
        self.cursplines = [Goto5(pcur, pd_final, move_time, space="task")]

        self.tstart = self.get_clock().now()

        # publish the goal point
        markermsg = Marker()
        markermsg.header.frame_id = "/base_link"
        markermsg.header.stamp = self.get_clock().now().to_msg()
        markermsg.type = 2
        markermsg.id = 0
        markermsg.scale.x = 0.05
        markermsg.scale.y = 0.05
        markermsg.scale.z = 0.05
        markermsg.color.r = 0.1
        markermsg.color.g = 1.0
        markermsg.color.b = 0.0
        markermsg.color.a = 1.0
        markermsg.pose.position.x = msg.x
        markermsg.pose.position.y = msg.y
        markermsg.pose.position.z = msg.z
        self.pub_goalmarker.publish(markermsg)


    # callback for command timer
    def cb_sendcmd(self):
        t = self.get_clock().now()
        dt = 1/RATE
        # default to holding position
        poscmd = [float("NaN"), float("NaN"), float("NaN")]
        velcmd = [float("NaN"), float("NaN"), float("NaN")]
        effcmd = [float("NaN"), float("NaN"), float("NaN")] # TODO: REPLACE WITH GRAVITY MODEL

        # check if there is a spline to run
        if self.cursplines:
            curspline = self.cursplines[0]
            # check if completed
            deltat = (t - self.tstart).nanoseconds / 1000000000.
            if (curspline.completed(deltat)):
                # Hold!
                self.cursplines.pop()
                self.tstart = self.get_clock().now()
                pass
            else:
                # do different things depending on the space
                if (curspline.space == 'joint'):
                    [qd, qd_dot] = curspline.evaluate(deltat)

                    poscmd = list(qd.reshape([3]))
                    velcmd = list(qd_dot.reshape([3])) 
                    effcmd = [float("NaN"), float("NaN"), float("NaN")] # TODO: REPLACE WITH GRAVITY MODEL
                    print(poscmd, velcmd)
                elif (curspline.space == 'task'):
                    [xd, xd_dot] = curspline.evaluate(deltat)

                    # compute forward kinematics
                    [xcurr, _, Jv, _] = self.chain.fkin(self.q)

                    # get qdot with J qdot = xdot
                    ex = xd - xcurr
                    qd_dot = np.linalg.pinv(Jv) @ (xd_dot + LAM * ex)
                    qd = self.q + qd_dot * dt
                    
                    # save commands
                    poscmd = list(qd.reshape([3]))
                    velcmd = list(qd_dot.reshape([3]))
                    effcmd = [float("NaN"), float("NaN"), float("NaN")] # TODO: REPLACE WITH GRAVITY MODEL
                    print(poscmd, velcmd)
        else:
            # Hold!
            pass

        # Publish!
        cmdmsg = JointState()
        cmdmsg.header.stamp = t.to_msg()
        cmdmsg.name = self.jointnames
        cmdmsg.position = poscmd
        cmdmsg.velocity = velcmd
        cmdmsg.effort = effcmd
        self.pub_jtcmd.publish(cmdmsg)


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




def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = GotoNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()