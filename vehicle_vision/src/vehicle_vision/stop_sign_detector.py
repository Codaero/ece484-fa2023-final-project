#!/usr/bin/env python
import rospy

import cv2
import numpy as np

#from sensor_msgs import Image

from ultralytics import YOLO

OBJECT_OF_INTEREST = "stop sign"


class StopSignDetector():

    def __init__(self, name):
        # Load a model
        model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)


    def detect_stop_sign():
        for detection in results:
            # confidence = np.argmax(detection.probs)
            # print(confidence)
            # print(detection.boxes)
            
            for c in detection.boxes.cls:
                detection_name = model.names[int(c)]

                if detection_name == OBJECT_OF_INTEREST:
                    print("Stop Sign Detected")
                    return True
        return False