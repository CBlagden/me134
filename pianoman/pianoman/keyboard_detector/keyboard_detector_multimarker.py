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
from geometry_msgs.msg import Pose, Point, Quaternion

import pianoman.utils.TransformHelpers as TransformHelpers


POINTS = np.array([
    (0.0, 1.4),
    (0.6, 1.4),
    (0.6, 0),
    (0, 0)
]).reshape(4, 1, 2).astype(np.float32)


POINTS_3D = np.array([
    (0.0, 1.4, 0.0),
    (0.6, 1.4, 0.0),
    (0.6, 0.0, 0.0),
    (0.0, 0.0, 0.0)
]).reshape(4, 1, 3).astype(np.float32)

Z_OFFSET = 0.065 # cm
PUBLISH_RATE = .01
CORNER_IDS = [0, 1, 2, 3]
KEYBOARD_IDS = [4, 5] # TODO: define actual keyboard id - and this should support multiple tags to account for accidental occlusions

CORNER_OFFSET = np.array([
    # -0.016 , -0.011, 0.0
    0.0, 0.0, 0.0
]).reshape((3, 1))

POINT_AVERAGE_OFFSET = np.array([
  0.833/2, 0.165/2, 0.0
]).reshape((3, 1))

KB_MARKER_ANGLE_OFFSET = -0.1082


def undistorted_to_world(coords_undistorted, rvec, tvec, z_offset):
    # we get the normalized image coordinate from undistortPoints
    normalized_point = list(coords_undistorted.flatten())

    # we make the point homogeneous
    normalized_point = np.array([*normalized_point, 1.0]).reshape((3, 1))

    Rot, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(Rot)

    # we solve the left and right hand sides of the equation for lambda
    left_side = R_inv @ normalized_point
    right_side = R_inv @ tvec
    lambda_ = (z_offset + right_side[2][0]) / left_side[2][0]

    # once we have lambda_ we can project from normalized image coordinates to
    # world coordinates
    point_w = R_inv @ (lambda_ * normalized_point - tvec)
    point_w = point_w + CORNER_OFFSET

    return point_w

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

        self.pub_kb_pos = self.create_publisher(Pose, name+"/keyboard_point", 3)
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

        self.rot = None

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

                if id in KEYBOARD_IDS:
                    if topLeft[1] != topRight[1] or topLeft[0] != bottomLeft[0]:
                        delta_y = topLeft[0] - bottomLeft[0]
                        delta_x = bottomLeft[1] - topLeft[1]
                        rot = -np.arctan2(delta_y, delta_x)
                    else:
                        rot = 0

                # Draw the box around the marker.
                pts = np.array([box.reshape(-1,1,2).astype(int)])
                cv2.polylines(frame, pts, True, self.green, 2)

                # Draw the center circle and write the ID there.
                ctr = tuple(center.astype(int))
                cv2.circle(frame, ctr, 4, self.red, -1)
                cv2.circle(frame, tuple(bottomLeft.astype(int)), 3, self.blue, -1)
                cv2.putText(frame, str(id), ctr,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 2)

            marker_id_to_idx = {}
            markers = {}
            corner_markers = {}
            keyboard_markers = {}
            for idx, (box, id) in enumerate(zip(boxes, ids.flatten())):
                marker_id_to_idx[id] = idx
                markers[id] = box[0]
                if id in CORNER_IDS:
                    corner_markers[id] = box[0]
                elif id in KEYBOARD_IDS:
                    keyboard_markers[id] = box[0]

            if len(corner_markers) == 4:
                bottom_lefts = [corners[3] for corners in corner_markers.values()]
                self.get_logger().info("Four corners detected... recomputing perspective transform")
                corner_markers_image_points = np.array(list(bottom_lefts)).reshape(4, 1, 2)
                corresponding_points = POINTS[list(corner_markers.keys())]
                corresponding_points_3D = POINTS_3D[list(corner_markers.keys())]

                coords = cv2.undistortPoints(corner_markers_image_points, self.camK, self.camD)
                self.M = cv2.getPerspectiveTransform(coords, corresponding_points)

                _, self.rvec, self.tvec = cv2.solvePnP(corresponding_points_3D, corner_markers_image_points, self.camK, self.camD)


            if self.M is not None:
                bottom_lefts = [corners[3] for corners in markers.values()]
                bottom_lefts_arr = np.array(bottom_lefts).reshape(len(boxes), 1, 2)

                # we get the normalized image coordinates
                coords_undistorted = cv2.undistortPoints(bottom_lefts_arr, self.camK, self.camD)

                # for key, val in corner_markers.items():
                #     if ids[key] not in KEYBOARD_IDS:
                #         point = list(undistorted_to_world(coords_undistorted[key], self.rvec, self.tvec, Z_OFFSET).flatten())
                #         self.get_logger().info(f"marker {ids[key]}: point {point}")

                # To print perspective comparisons:
                # we make the normalized image coordinates homogeneous
                coords = np.concatenate([coords_undistorted, np.ones((len(bottom_lefts_arr), 1, 1))], axis=-1)
                # we multiply by the perspective transform to obtain the world coordinates
                coords = coords @ self.M.T

                # if len(keyboard_markers) == 1:
                #     kb_marker_id, kb_marker_coords = list(keyboard_markers.items())[0]
                #     kb_marker_idx = marker_id_to_idx[kb_marker_id]
                #     keyboard_coords_undistorted = cv2.undistortPoints(kb_marker_coords, self.camK, self.camD)
                #     (topLeft, topRight, _, bottomLeft) = [(keyboard_coords_undistorted[i][0][0], keyboard_coords_undistorted[i][0][1]) for i in range(4)]
                #     if topLeft[1] != topRight[1] or topLeft[0] != bottomLeft[0]:
                #         delta_y = topLeft[0] - bottomLeft[0]
                #         delta_x = bottomLeft[1] - topLeft[1]
                #         rot = -np.arctan2(delta_y, delta_x)
                #     else:
                #         rot = 0
                if len(keyboard_markers) == 2:
                    self.get_logger().info("Two detected!")
                    marker_1, marker_2 = keyboard_markers[KEYBOARD_IDS[0]], keyboard_markers[KEYBOARD_IDS[1]]
                    marker_1_undistorted = cv2.undistortPoints(marker_1, self.camK, self.camD)[0]
                    marker_2_undistorted = cv2.undistortPoints(marker_2, self.camK, self.camD)[0]
                    center_1 = np.array([(np.mean(marker_1_undistorted[:, 0])), float(np.mean(marker_1_undistorted[:, 1]))]).reshape(1, 2)
                    center_2 = np.array([float(np.mean(marker_2_undistorted[:, 0])), float(np.mean(marker_2_undistorted[:, 1]))]).reshape(1, 2)
                    center_1_world = undistorted_to_world(center_1, self.rvec, self.tvec, Z_OFFSET)
                    center_2_world = undistorted_to_world(center_2, self.rvec, self.tvec, Z_OFFSET)
                    delta_y = center_2_world[1][0] - center_1_world[1][0]
                    delta_x = center_2_world[0][0] - center_1_world[0][0]
                    self.rot = np.arctan2(delta_x, -delta_y) + KB_MARKER_ANGLE_OFFSET

                    c1 = np.array((float(np.mean(marker_1[:, 0])), float(np.mean(marker_1[:, 1]))))
                    next_pt = c1 + np.array([500 * np.cos(self.rot), -500 * np.sin(self.rot)])
                    pts = np.array([np.vstack([c1, next_pt]).astype(int)])
                    cv2.polylines(frame, pts, True, self.white, 2)


                if KEYBOARD_IDS[0] in keyboard_markers.keys() and KEYBOARD_IDS[1] in keyboard_markers.keys():
                    kb_marker_idx = marker_id_to_idx[KEYBOARD_IDS[0]]

                    self.get_logger().info(str("rotation (world): " + str(np.degrees(self.rot))))
                    point_w = list(undistorted_to_world(coords_undistorted[marker_id_to_idx[KEYBOARD_IDS[0]]], self.rvec, self.tvec, Z_OFFSET).flatten())

                    Rot_mat = TransformHelpers.Rotz(self.rot)

                    point_kb_origin = list(undistorted_to_world(coords_undistorted[marker_id_to_idx[KEYBOARD_IDS[0]]], self.rvec, self.tvec, Z_OFFSET).flatten())
                    point_kb_opposite = list(undistorted_to_world(coords_undistorted[marker_id_to_idx[KEYBOARD_IDS[1]]], self.rvec, self.tvec, Z_OFFSET).flatten())

                    point_w_mean = np.mean([np.array(point_kb_origin), np.array(point_kb_opposite)], axis=0).reshape((3, 1))
                    point_w_mean = point_w_mean - TransformHelpers.Rotz(-np.pi/2) @ Rot_mat @ POINT_AVERAGE_OFFSET
                    point_w = list(point_w_mean.flatten())

                    point = Point(x=point_w[0],
                                    y=point_w[1],
                                    z=point_w[2])
                    pose = Pose()
                    pose.position = point
                    quat = list(TransformHelpers.quat_from_R(Rot_mat))
                    q = Quaternion(x=quat[0],
                                    y=quat[1],
                                    z=quat[2],
                                    w=quat[3])
                    pose.orientation = q

                    # # using perspective transform to get point
                    point = coords[kb_marker_idx].flatten()
                    x_c = point[0] / point[2]
                    y_c = point[1] / point[2]

                    self.get_logger().info("solvePnP point " + str(point_w))
                    self.get_logger().info("perspective point " + str(point))

                    self.pub_kb_pos.publish(pose)

            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
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
