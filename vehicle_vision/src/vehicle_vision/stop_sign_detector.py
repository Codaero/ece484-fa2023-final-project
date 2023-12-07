#!/usr/bin/env python
import rospy

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO


OBJECT_OF_INTEREST = "stop sign"

CAMERA_TOPIC = "/zed2/zed_node/right_raw/image_raw_color"


class StopSignDetector():

    def __init__(self):
        # Load a model
        self.model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
        self.sub_image = rospy.Subscriber(CAMERA_TOPIC, Image, self.img_callback_real, queue_size=1)
        self.stop_sign_detected = False;
        self.bridge = CvBridge()

    def img_callback_real(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model.predict(cv_image)

        for detection in results:            
            for c in detection.boxes.cls:
                detection_name = self.model.names[int(c)]

                if detection_name == OBJECT_OF_INTEREST:
                    print("Stop Sign Detected")
                    self.stop_sign_detected = True
        self.stop_sign_detected = False


    def detect_stop_sign(self):
        return self.stop_sign_detected