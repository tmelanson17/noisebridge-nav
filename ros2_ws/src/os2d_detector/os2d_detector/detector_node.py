import sys
import os
import queue
import rclpy
import cv2
import PIL
import numpy as np
import cv_bridge

from rclpy.node import Node
from sensor_msgs import msg

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from os2d.inference import OS2DDetector


class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.image_queue = queue.Queue()
        self.subscription = self.create_subscription(
            msg.Image, "/oakd/rgb/preview/image_raw", self.listener_callback, 10
        )
        self.subscription
        self.publisher = self.create_publisher(
            msg.Image, "/os2d/detection_output", 10
        )
        self.model = OS2DDetector()
        self.bridge = cv_bridge.CvBridge()

    def _imgmsg_to_cv2(self, img_msg):
        n_channels = 3
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        img_buf = (
            np.asarray(img_msg.data, dtype=dtype)
            if isinstance(img_msg.data, list)
            else img_msg.data
        )
        im = np.ndarray(
            shape=(
                img_msg.height,
                int(img_msg.step / dtype.itemsize / n_channels),
                n_channels,
            ),
            dtype=dtype,
            buffer=img_buf,
        )
        im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])

        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        return im

    def listener_callback(self, msg):
        image = self._imgmsg_to_cv2(msg)
        im_h, im_w, _ = image.shape

        # res = self.model.predict(PIL.Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))

        # boxes = [
        #     [
        #         (int(box[0] * im_w), int(box[1] * im_h)),
        #         (int(box[2] * im_w), int(box[3] * im_h)),
        #     ]
        #     for box in res["bboxes"]
        # ]
        # for box in boxes:
        #     image = cv2.rectangle(image, box[0], box[1], (255, 0, 0), 2)
        # Random rectangle added for timing purposes
        image = cv2.rectangle(image, (0,0),(50,50),(255,0,0),2)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CameraImageSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
