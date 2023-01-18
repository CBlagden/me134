'''hw4p4.py

   This is the solution code for HW4 Problem 4.

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

from hwsols.Segments    import Hold, Stay, GotoCubic, SplineCubic
from hwsols.hw4p3sol    import fkin, Jac


# List of seven targets.
targetpoints = [np.array([0.5,  1.0, 0.5]).reshape((3,1)), 
                np.array([0.5,  2.0, 0.5]).reshape((3,1)),
                np.array([1.0,  0.5, 0.5]).reshape((3,1)),
                np.array([1.0,  0.0, 0.5]).reshape((3,1)),
                np.array([0.0, -1.2, 0.5]).reshape((3,1)),
                np.array([0.2, -1.0, 0.5]).reshape((3,1)),
                np.array([0.5, -1.0, 0.5]).reshape((3,1))]

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self):
        # Build up the list of segments, starting with nothing.
        self.segments = []

        # For each target (goal position), add the visualizing movements.
        for xgoal in targetpoints[0:]:
            # Set the initial joint value guess.
            q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)
            
            # Start the visualization holding for 1s at q(0)
            # This wasn't asked in the HW, but makes it look better.
            self.segments.append(Hold(q, 1.0))

            # Newton-Raphson Algorithm: Iterate seven times.
            for k in range(7):
                x = fkin(q)
                J = Jac(q)
                qnext = q + np.linalg.inv(J) @ (xgoal - x)

                self.segments.append(GotoCubic(q, qnext, 1.0))
                q = qnext

            # End the visualization holding for 1s at q(7)
            # This also wasn't asked in the HW, but makes it look better.
            self.segments.append(Hold(q, 1.0))
            
            # Print the result.
            print("Target xgoal = ", xgoal.T, " -> q(7) = ", q.T)

        # Hold at the last point indefinitely.  Making cyclic pointless.
        self.segments.append(Stay(q))
        self.cyclic = False
    
        # Zero the start time of the current segment.
        self.t0 = 0.0


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, tabsolute):
        # Check whether the segment is done.
        if self.segments[0].completed(tabsolute - self.t0):
            # If the current segment is done, shift to the next
            self.t0 = self.t0 + self.segments[0].duration()
            seg = self.segments.pop(0)
            if self.cyclic:
                self.segments.append(seg)

        # Compute the positions/velocities as a function of time.
        (q, qdot) = self.segments[0].evaluate(tabsolute - self.t0)

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


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
