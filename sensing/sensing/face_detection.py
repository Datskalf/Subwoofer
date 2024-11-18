import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CompressedImage
from interfaces.msg import FacePoint, FacePoints

from cv_bridge import CvBridge

import cv2
import os
import pathlib

class FaceDetection(Node):
    #CASCADE_PATH: str = "/home/sondre/subwoofer/src/sensing/sensing/haarcascade_frontalface_default.xml"
    CASCADE_PATH: str = os.path.join(
        pathlib.Path(__file__).parent.absolute(),
        "haarcascade_frontalface_default.xml"
    )

    def __init__(self):
        super().__init__("face_detection")
        self.get_logger().info(f"Cascade path: {self.CASCADE_PATH}")
        self.cascade = cv2.CascadeClassifier(self.CASCADE_PATH)

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.detect_face,
            10
        )

        self.face_detect_pub = self.create_publisher(
            FacePoints,
            "subwoofer/face_detector",
            10
        )

        self.get_logger().info(f"Face detector online!")


    def detect_face(self, msg) -> None:
        #self.get_logger().info(f"Running face detection")
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        rects = self.cascade.detectMultiScale(
            img,
            scaleFactor=1.3,
            minNeighbors=4,
            minSize=(30,30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        if len(rects) == 0:
            return
        
        self.get_logger().info(f"{rects[0]}")

        msg = FacePoints()
        for rect in rects:
            r = FacePoint()
            r.x = int(rect[0])
            r.y = int(rect[1])
            r.width = int(rect[2])
            r.height = int(rect[3])
            msg.points.append(r)
        self.face_detect_pub.publish(msg)
        ...




def main(args=None):
    rclpy.init(args=args)
    node = FaceDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()