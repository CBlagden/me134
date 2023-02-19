#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Detect the ArUco marker with OpenCV.
#
#   Node:           /keyboarddetector
#   Subscribers:    /camera/color/image_raw          Source image
#   Publishers:     /keyboarddetector/image_raw      Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge
from sensor_msgs.msg import CameraInfo

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from geometry_msgs.msg import Point


POINTS = np.array([
    (0.001, 1.227),
    (0.566, 1.353),
    (0.571, 0),
    (0, 0)
]).reshape(4, 1, 2).astype(np.float32)


POINTS_3D = np.array([
    (0.001, 1.227, 0),
    (0.566, 1.353, 0),
    (0.571, 0, 0),
    (0, 0, 0)
]).reshape(4, 1, 3).astype(np.float32)

Z_OFFSET = 0.06 # cm
PUBLISH_RATE = .01
KEYBOARD_ID = 4 # TODO: define actual keyboard id - and this should support multiple tags to account for accidental occlusions

#
#  Detector Node Class
#
class KeyboardNode(Node):
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

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Set up the ArUco detector.  Use a dictionary with the
        # appropriate tags.  DICT_6x6_1000 has 1000 options (way too
        # many).  DICT_6x w/6_250 has 250 options.  DICT_6x6_50 is
        # probably enough for our projects...
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # Report. 
        self.get_logger().info("Keyboard detector running...")

        # Get camera info
        self.get_logger().info("Waiting for camera info...")
        sub = self.create_subscription(CameraInfo, '/camera_info', self.cb_get_cam_info, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        self.pub_kb_pos = self.create_publisher(Point, name+"/keyboard_point", 3)
        self.M = None

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

            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

            corner_markers = {}
            for (box, id) in zip(boxes, ids.flatten()):
                if 0 <= id <= 3:
                    if id == 0 or id == 1:
                        # save the bottom left point from each detected table corner
                        corner_markers[id] = box[0][3]
                    else:
                        # save the bottom right point for these tags
                        corner_markers[id] = box[0][2]

            if len(corner_markers) == 4:
                self.get_logger().info("Four corners detected... recomputing perspective transform")
                corner_markers_image_points = np.array(list(corner_markers.values())).reshape(4, 1, 2)
                corresponding_points = POINTS[list(corner_markers.keys())]
                corresponding_points_3D = POINTS_3D[list(corner_markers.keys())]

                coords = cv2.undistortPoints(corner_markers_image_points, self.camK, self.camD)
                self.M = cv2.getPerspectiveTransform(coords, corresponding_points)

                _, self.rvec, self.tvec = cv2.solvePnP(corresponding_points_3D, corner_markers_image_points, self.camK, self.camD)


            if self.M is not None:
                bottom_lefts = np.array([box[0][3] for box in boxes]).reshape(len(boxes), 1, 2)

                # we get the normalized image coordinates
                coords_undistorted = cv2.undistortPoints(bottom_lefts, self.camK, self.camD)

                # we make the normalized image coordinates homogeneous
                coords = np.concatenate([coords_undistorted, np.ones((len(bottom_lefts), 1, 1))], axis=-1)

                # we multiple by the perspective transform to obtain the world coordinates
                coords = coords @ self.M.T

                for i, id in enumerate(ids):
                    if id == KEYBOARD_ID:
                        # we get the normalized image coordinate from undistortPoints
                        normalized_point = list(coords_undistorted[i].flatten())

                        # we make the point homogeneous
                        normalized_point = np.array([*normalized_point, 1.0]).reshape((3, 1))

                        Rot, _ = cv2.Rodrigues(self.rvec)
                        R_inv = np.linalg.inv(Rot)

                        # we solve the left and right hand sides of the equation for lambda
                        left_side = R_inv @ normalized_point
                        right_side = R_inv @ self.tvec
                        lambda_ = (Z_OFFSET + right_side[2][0]) / left_side[2][0]

                        # once we have lambda_ we can project from normalized image coordinates to
                        # world coordinates
                        point_w = R_inv @ (lambda_ * normalized_point - self.tvec)

                        point_w = list(point_w.flatten())
                        p = Point()
                        p.x = point_w[0]
                        p.y = point_w[1]
                        p.z = point_w[2]

                        # # using perspective transform to get point
                        point = coords[i].flatten()
                        x_c = point[0] / point[2]
                        y_c = point[1] / point[2]

                        self.get_logger().info("solvePnP point " + str(point_w))
                        self.get_logger().info("perspective point " + str(point))

                        self.pub_kb_pos.publish(p)
                        break

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = KeyboardNode('keyboarddetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
