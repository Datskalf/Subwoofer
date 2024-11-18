import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from flask import Flask, render_template

import os
import pathlib

TEMPLATE_PATH = os.path.join(pathlib.Path(__file__).parent, "templates")


class WebGUI(Node):
    def __init__(self):
        super().__init__("web_gui")

        self.camera_feed: Image | None = None

        self.app = Flask(__name__, template_folder=TEMPLATE_PATH)
        self.add_routes()
        self.app.run(debug=True)



        # Subscribers
        self.camera_feed_subscriber = self.create_subscription(
            Image,
            "/image_raw/compressed",
            self.camera_feed_callback,
            10
        )

    def camera_feed_callback(self, msg: Image) -> None:
        self.camera_feed = msg.data


    def add_routes(self) -> None:
        self.app.add_url_rule('/', 'hello_world', self.flask_hello_world)
        self.app.add_url_rule('/template', 'template', self.flask_render)

    def flask_hello_world(self) -> None:
        return f'Hello world!'
    
    def flask_render(self) -> None:
        return render_template("/home/sondre/subwoofer/src/user_interface/user_interface/templates/gui.html")
        ...


def main(args=None):
    rclpy.init(args=args)
    node = WebGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()