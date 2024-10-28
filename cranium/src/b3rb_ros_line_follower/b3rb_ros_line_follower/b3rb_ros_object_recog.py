import rclpy
from rclpy.node import Node
from synapse_msgs.msg import TrafficStatus

import cv2
import numpy as np
import torch

from sensor_msgs.msg import CompressedImage

QOS_PROFILE_DEFAULT = 10


class ObjectRecognizer(Node):
    """Initializes object recognizer node with the required publishers and subscriptions."""
    def __init__(self):
        super().__init__('object_recognizer')

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or 'yolov5m', 'yolov5l', 'yolov5x'

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT)

        # Publisher for traffic status.
        self.publisher_traffic = self.create_publisher(
            TrafficStatus,
            '/traffic_status',
            QOS_PROFILE_DEFAULT)

    def camera_image_callback(self, message):
        # Convert message to an n-dimensional numpy array representation of image.
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform object detection
        results = self.model(image)

        # Extract detected objects
        detected_objects = set()
        for *box, conf, cls in results.xyxy[0]:  # results.xyxy[0] is a tensor with detection results
            label = self.model.names[int(cls)]
            detected_objects.add(label)

        # Prepare TrafficStatus message
        traffic_status_message = TrafficStatus()
        traffic_status_message.detected_objects = list(detected_objects)  # Assuming TrafficStatus.msg has a field detected_objects

        # Publish the message
        self.publisher_traffic.publish(traffic_status_message)


def main(args=None):
    rclpy.init(args=args)

    object_recognizer = ObjectRecognizer()

    rclpy.spin(object_recognizer)

    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_recognizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
