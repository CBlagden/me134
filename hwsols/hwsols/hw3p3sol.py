'''
hw3p3.py

   This is the solution code for HW3 Problem 3.

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self):
        pass

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        return ['pan', 'tilt']

    # Evaluate at the given time.
    def evaluate(self, t):
        # Compute the joint values.  We could build up numpy arrays as
        # position and velocity vectors, but just compute the values:
        theta_pan      =   np.pi/3 * np.sin(2*t)
        theta_dot_pan  = 2*np.pi/3 * np.cos(2*t)

        theta_tilt     =   np.pi/3 * np.sin(1*t) -   np.pi/9 * np.cos(6*t)
        theta_dot_tilt = 1*np.pi/3 * np.cos(1*t) + 6*np.pi/9 * np.sin(6*t)

        # Return the position and velocity as python lists.
        return ([theta_pan,     theta_tilt    ],
                [theta_dot_pan, theta_dot_tilt])


#
#   Generator Node Class
#
class Generator(Node):
    # Initialization.
    def __init__(self):
        # Initialize the node, naming it 'generator'
        super().__init__('generator')

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        self.get_logger().info("Waiting for a subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up a trajectory.
        self.trajectory = Trajectory()
        self.jointnames = self.trajectory.jointnames()

        # Create a timer to keep calculating/sending commands.
        self.starttime = self.get_clock().now()
        rate           = 100
        self.timer     = self.create_timer(1/float(rate), self.update)
        dt             = self.timer.timer_period_ns * 1e-9
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Update - send a new joint command every time step.
    def update(self):
        # Grab the current time.
        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9

        # Compute the desired joint positions and velocities for this time.
        (q, qdot) = self.trajectory.evaluate(t)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the node.
    rclpy.init(args=args)
    generator = Generator()

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
