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
    last_image = None
    #CASCADE_PATH: str = "/home/sondre/subwoofer/src/sensing/sensing/haarcascade_frontalface_default.xml"
    #CASCADE_PATH: str = os.path.join(
    #    pathlib.Path(__file__).parent.absolute(),
    #    "haarcascade_frontalface_default.xml"
    #)
    CASCADE_PATH: str = os.path.join(
        os.environ["SW_PATH"],
        "static",
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
            "/subwoofer/face_detector/points",
            10
        )

        self.face_box_pub = self.create_publisher(
            CompressedImage,
            "/subwoofer/face_detector/compressed",
            10
        )

        self.get_logger().info(f"Face detector online!")


    def detect_face(self, msg) -> None:
        #self.get_logger().info(f"Running face detection")
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.last_image = img
        rects = self.cascade.detectMultiScale(
            img,
            scaleFactor=1.3,
            minNeighbors=4,
            minSize=(30,30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        if len(rects) > 0:
            rects[:,2:] += rects[:,:2]
            msg1 = FacePoints()

            for x1, y1, x2, y2 in rects:
                r = FacePoint()
                r.x = int(abs((x2-x1)//2))
                r.y = int(abs((y2-y1)//2))
                r.width = int(abs(x2-x1))
                r.height = int(abs(y2-y1))
                msg1.points.append(r)
            
            self.face_detect_pub.publish(msg1)
            self.draw_boxes(img, rects, (0,255,0))

        msg2 = self.bridge.cv2_to_compressed_imgmsg(img)
        self.face_box_pub.publish(msg2)



    def draw_boxes(self, image, boxes: list[int], colour: tuple[int]) -> None:
        for x1, y1, x2, y2 in boxes:
            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)




def main(args=None):
    rclpy.init(args=args)
    node = FaceDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()