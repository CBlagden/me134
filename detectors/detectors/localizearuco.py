#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Detect the ArUco marker with OpenCV.
#
#   Node:           /detectaruco
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /detectaruco/image_raw      Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray

from rclpy.node         import Node
from sensor_msgs.msg    import Image


POINTS = np.array([(1, 1), (0, 0), (0, 1), (1, 0)]).reshape(4, 1, 2).astype(np.float32)
PUBLISH_RATE = .01

#
#  Detector Node Class
#
class ArucoNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Set up the ArUco detector.  Use a dictionary with the
        # appropriate tags.  DICT_6x6_1000 has 1000 options (way too
        # many).  DICT_6x6_250 has 250 options.  DICT_6x6_50 is
        # probably enough for our projects...
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # Report.
        self.get_logger().info("ArUco detector running...")

        # Get camera info
        self.get_logger().info("Waiting for camera info...")
        sub = self.create_subscription(CameraInfo, '/usb_cam/camera_info', self.cb_get_cam_info, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Localize aruco markers
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)
        self.get_logger().info("Trying to detect all markers...")
        sub = self.create_subscription(
            Image, '/image_raw', self.cb_localize, 1)
        self.localized = False
        while not self.localized:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        self.get_logger().info("Publishing image transform data...")
        self.pub_camera_transform = self.create_publisher(Float64MultiArray, '/cam_transform', 10)

        self.timer = self.create_timer(PUBLISH_RATE, self.cb_publish_cam_transform)


    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def cb_get_cam_info(self, msg):
        self.camD = np.array(msg.d).reshape(5)
        self.camK = np.array(msg.k).reshape((3,3))
        self.caminfoready = True

    # Process the image (detect the aruco).
    def cb_localize(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Detect.
        (boxes, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.dict, parameters=self.params)

                # Loop over each marker: the list of corners and ID for this marker.
        if len(boxes) > 0:
            for (box, id) in zip(boxes, ids.flatten()):

                # The corners are top-left, top-right, bottom-right,
                # bottom-left of the original marker (not the bounding
                # box).  Each is a 2x1 numpy array of pixel
                # coordinates.  Their type is floating-point though
                # they contain integer values.
                (topLeft, topRight, bottomRight, bottomLeft) = box[0]
                center = (topLeft + bottomRight)/2

                # Draw the box around the marker.
                pts = [box.reshape(-1,1,2).astype(int)]
                cv2.polylines(frame, pts, True, self.green, 2)
            
                # Draw the center circle and write the ID there.
                ctr = tuple(center.astype(int))
                cv2.circle(frame, ctr, 4, self.red, -1)
                cv2.putText(frame, str(id), ctr,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 2)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        if len(boxes) == 4 and self.caminfoready:
            self.get_logger().info("Markers detected.")

            bottom_lefts = np.array([box[0][3] for box in boxes]).reshape(4, 1, 2)
            coords = cv2.undistortPoints(bottom_lefts, self.camK, self.camD)
            self.M = cv2.getPerspectiveTransform(coords, POINTS)
            self.M = self.M.flatten()
            self.M = [float(v) for v in self.M]
            self.localized = True
                
    def cb_publish_cam_transform(self):
        msg = Float64MultiArray()
        self.pub_camera_transform.publish(Float64MultiArray(data=self.M))

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = ArucoNode('localizearuco')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
