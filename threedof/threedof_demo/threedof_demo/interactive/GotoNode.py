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
from threedof_demo.Segments import Goto5, Hold, Stay

from sensor_msgs.msg    import JointState
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, String

from threedof_demo.analytic_solver import get_sol
from threedof_demo.rviz_helper import create_pt_marker

RATE = 100.0
LAM = 10

MOVE_TIME = 5
NUM_LINE_REPETITIONS = 20

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
        self.pre_flip_dt = None
        # general
        self.cursplines = []
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]

        ## Publishers
        self.pub_jtcmd = self.create_publisher(\
                JointState, '/joint_commands', 10)
        self.pub_goalmarker = self.create_publisher(\
                Marker, '/goal_marker', 10)
        self.pub_cartcmd = self.create_publisher(\
                PointStamped, '/cart_commands', 10)
        self.pub_cartstate = self.create_publisher(\
                PointStamped, '/cart_states', 10)

        ## Subscribers
        self.sub_jtstate = self.create_subscription(\
                JointState, '/joint_states', self.cb_jtstate, 10)
        self.sub_pointcmd = self.create_subscription(\
                Point, '/point_cmd', self.cb_pointcmd, 10)
        self.sub_linecmd = self.create_subscription(\
                String, '/line_cmd', self.cb_linecmd, 10)
        self.sub_flipcmd = self.create_subscription(Empty, '/flip_cmd', self.cb_flip, 1)

        ## Timers
        self.cmdtimer = self.create_timer(1 / RATE, self.cb_sendcmd)

        self.a = -.1275
        self.b = 0.
        self.c = 2.5
        self.d = 0.

        self.mode = 'pan_forward'
        self.flipping = False

    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint posiitons
        self.q = np.array(msg.position).reshape([3,1])
        self.qdot = np.array(msg.velocity).reshape([3,1])

    
    # callback for /point_cmd
    def cb_pointcmd(self, msg):
        # reset spline list
        self.cursplines = []
        # Use this to initiate/reset splines between cartesian positions
        pd_final = [msg.x, msg.y, msg.z]
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        pd_final = np.array(pd_final).reshape([3,1])
        
        self.cursplines.append(Goto5(pcur, pd_final, MOVE_TIME, space='task'))
        # self.cursplines.append(Hold(pd_final, MOVE_TIME, space='task'))
        self.cursplines.append(Stay(pd_final, space='task'))
        self.tstart = self.get_clock().now()

        # publish the goal point
        markermsg = create_pt_marker(msg.x, msg.y, msg.z, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg)

    # callback for /line_cmd
    def cb_linecmd(self, msg):
        # reset spline list
        self.cursplines = []
        # Use this to initiate/reset splines between cartesian positions
        x1, y1, z1, x2, y2, z2 = map(float, msg.data.split('x'))
        pd1_final = [x1, y1, z1]
        pd2_final = [x2, y2, z2]
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final
        pcur = np.array(pcur).reshape([3,1])
        pd1_final = np.array(pd1_final).reshape([3,1])
        pd2_final = np.array(pd2_final).reshape([3,1])

        self.tstart = self.get_clock().now()
        self.cursplines.append(Goto5(pcur, pd1_final, MOVE_TIME, space='task'))
        for _ in range(NUM_LINE_REPETITIONS):
            self.cursplines.append(Goto5(pd1_final, pd2_final, MOVE_TIME, space='task'))
            self.cursplines.append(Goto5(pd2_final, pd1_final, MOVE_TIME, space='task'))
        self.cursplines.append(Stay(pd1_final, space='task'))

        # publish the goal point
        markermsg1 = create_pt_marker(x1, y1, z1, self.get_clock().now().to_msg())
        markermsg2 = create_pt_marker(x2, y2, z2, self.get_clock().now().to_msg())
        self.pub_goalmarker.publish(markermsg1)
        self.pub_goalmarker.publish(markermsg2)

    # callback for /flip_cmd
    def cb_flip(self, _):
        xcurr, _, _, _ = self.chain.fkin(self.q)
        x = float(xcurr[0, 0])
        y = float(xcurr[1, 0])
        z = float(xcurr[2, 0])
        self.get_logger().info("Flipping...")
        if self.mode == 'pan_forward':
            jt_f = get_sol(x, y, z, 'pan_backward')
            self.mode = 'pan_backward'
        elif self.mode == 'pan_backward':
            jt_f = get_sol(x, y, z, 'pan_forward')
            self.mode = 'pan_forward'
        jt_f = np.array(jt_f).reshape([3,1])
        self.cursplines.insert(0, Goto5(self.q, jt_f, MOVE_TIME, space='joint'))

        self.pre_flip_dt = self.get_clock().now() - self.tstart
        self.tstart = self.get_clock().now()
        self.flipping = True

    def cb_sendcmd(self):
        t = self.get_clock().now()
        dt = 1/RATE
        # default to holding position
        poscmd = [float("NaN"), float("NaN"), float("NaN")]
        velcmd = [float("NaN"), float("NaN"), float("NaN")]
        effcmd = self.gravity()

        # check if there is a spline to run
        if (self.cursplines):
            # check if completed
            curspline = self.cursplines[0]
            deltat = (t - self.tstart).nanoseconds / 1000000000.
            if (curspline.completed(deltat)):
                if self.flipping:
                    self.flipping = False
                    self.tstart = t - self.pre_flip_dt
                else: self.tstart = t
                self.cursplines.pop(0)
            else:
                # do different things depending on the space
                if (curspline.get_space() == 'joint'):
                    [qd, qd_dot] = curspline.evaluate(deltat)

                    poscmd = list(qd.reshape([3]))
                    velcmd = list(qd_dot.reshape([3])) 
                    print(poscmd, velcmd)
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

                    # Publish joint states
                    msg = PointStamped()
                    msg.header.stamp = t.to_msg()
                    msg.point.x = xd[0,0]
                    msg.point.y = xd[1,0]
                    msg.point.z = xd[2,0]
                    self.pub_cartcmd.publish(msg)

                    msg2 = PointStamped()
                    msg2.header.stamp = t.to_msg()
                    msg2.point.x = xcurr[0,0]
                    msg2.point.y = xcurr[1,0]
                    msg2.point.z = xcurr[2,0]
                    self.pub_cartstate.publish(msg2)
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
        _, t1, t2 = self.q
        tau1 = self.a * np.sin(-t1 + t2) + self.b * np.cos(-t1 + t2) + self.c * np.sin(-t1) + self.d * np.cos(-t1)
        tau2 = self.a * np.sin(-t1 + t2) + self.b * np.cos(-t1 + t2)
        return [0., float(tau1), float(tau2)]
    
    def log(self, to_log):
        self.get_logger().info(to_log)
    
    def log_pos(self):
        self.log("pos: %f, %f, %f" % (self.q[0] * 180/np.pi, self.q[1] * 180/np.pi, self.q[2] * 180/np.pi))

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