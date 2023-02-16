import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        def cb(msg):
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(mdg.k).reshape((3, 3))
            self.camw = msg.width
            self.camh = msg.height
            self.caminfoready = True

        sub = self.create_subscription(CameraInfo, '/usb_cam/camera_info', cb, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)


    
