#!/usr/bin/env python
import rospy

import cv2
import numpy as np

import torch
from torchvision import transforms

from sensor_msgs.msg import Image as sensor_msgs_Image
from cv_bridge import CvBridge, CvBridgeError

from PIL import Image

# Topics:
CAMERA_TOPIC = "/zed2/zed_node/right_raw/image_raw_color"
LANE_OVERLAY_TOPIC = "/lane_det/lane_overlay"

class LaneDetNode:
    def __init__(self) -> None:
        # Initialize PyTorch model and CUDA device
        self.model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True)
        self.device = torch.device("cuda")
        
        # Setup PyTorch model
        self.model.to(self.device)
        self.model.eval()
        
        self.bridge = CvBridge()
        
        # Initialize Subscriber
        self.zed2_sub = rospy.Subscriber(CAMERA_TOPIC, sensor_msgs_Image, self.zed2_image_callback, queue_size=1)

        # Initialize Publisher
        self.overlay_publisher = rospy.Publisher(LANE_OVERLAY_TOPIC, sensor_msgs_Image, queue_size=1)
    
    def zed2_image_callback(self, data: sensor_msgs_Image) -> None:
        """
        Processes images from Zed2 camera and publishes inference from PyTorch model

        - data: 1 sensor_msgs/Image message from Zed2 Camera
        - return: None
        """ 
        
        # Convert ROS message to an openCV data type
        cv2_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        
        alpha = 1.5
        beta = 1.5
        
        cv2_img = cv2.convertScaleAbs(cv2_img, alpha=alpha, beta=beta)
        # cv2.imshow("original image", cv2_img)
        
        # cv2.imshow("adjusted image", cv2_img_adj)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # while True:
        #     pass
        # apply padding to align to multiple of 32 before feeding into model
        padding_rows = 16
        padding_color = [0,0,0] # add black pixels to bottom of image
        
        cv2_img = cv2.copyMakeBorder(cv2_img, 0, padding_rows, 0, 0, cv2.BORDER_CONSTANT, value=padding_color)
        
        # Define the standard normalization transformation
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((736, 1280)),  # Resize the image to a valid input size
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        img_tensor = transform(cv2_img).to(self.device)
        # Add a batch dimension
        img_tensor = img_tensor.unsqueeze(0)

        # Inference
        with torch.no_grad():
            det_out, da_seg_out, ll_seg_out = self.model(img_tensor)

        
        ll_seg_mask = torch.argmax(ll_seg_out, 1)
        ll_seg_mask = ll_seg_mask.squeeze(0).cpu().numpy()

        # Assuming 'img' is normalized in the range [0, 1]
        # Convert it to a PIL image for visualization
        # If 'img' is not normalized, you should normalize it as per the model's requirements
        img_tensor = img_tensor.squeeze(0).permute(1, 2, 0)  # Change dimension from [C, H, W] to [H, W, C]
        img_tensor = (img_tensor * 255).byte().cpu().numpy()  # Denormalize and convert to numpy
        
                
        img_pil = Image.fromarray(img_tensor)

        # Create an overlay mask for the lanes
        lane_overlay = np.zeros_like(img_tensor, dtype=np.uint8)
        lane_overlay[ll_seg_mask == 1] = [255, 0, 0]  # Assuming class '1' corresponds to lane lines

        # Crop Image back to original resolution:
        lane_overlay = lane_overlay [:720, :]
        
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(lane_overlay, 'rgb8'))
        
        # Overlay the binary mask onto the original image
        # Adjust the alpha parameter to control the transparency of the overlay
        # alpha = 0.5
        # img_with_lanes = Image.blend(img_pil, Image.fromarray(lane_overlay), alpha=alpha)

        # print("inference done")

if __name__ == '__main__':
    rospy.init_node('lane_det_node')
    LaneDetNode()
    rospy.spin()