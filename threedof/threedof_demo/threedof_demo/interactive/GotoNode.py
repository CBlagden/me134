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

from sensor_msgs.msg    import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

RATE = 20.0
LAM = 10

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

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)

        self.a = -.1275
        self.b = 0.
        self.c = 2.6
        self.d = 0.


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
        move_time = 5 #s
        self.cursplines.append(Goto5(pcur, pd_final, move_time, space="task"))

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
        effcmd = [float("NaN"), float("NaN"), float("NaN")]

        # check if there is a spline to run
        if (self.cursplines):
            curspline = self.cursplines[0]
            # check if completed
            deltat = (t - self.tstart).nanoseconds / 1000000000.
            if (curspline.completed(deltat)):
                self.cursplines.pop()
                self.tstart = self.get_clock().now()
            else:
                # do different things depending on the space
                if (curspline.get_space() == 'joint'):
                    # TODO (@JOAQUIN!!)
                    pass
                elif (curspline.get_space() == 'task'):
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
                    print(poscmd, velcmd)
        else:
            effcmd = self.gravity()

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

    def gravity(self):
        _, t1, t2 = list(self.q.reshape(3))
        tau1 = self.a * np.sin(-t1 + t2) + self.b * np.cos(-t1 + t2) + self.c * np.sin(-t1) + self.d * np.cos(-t1)
        tau2 = self.a * np.sin(-t1 + t2) + self.b * np.cos(-t1 + t2)
        return [0., float(tau1), float(tau2)]

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