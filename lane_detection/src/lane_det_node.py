#!/usr/bin/env python
import rospy

import cv2
import numpy as np

import torch
from torchvision import transforms

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line

from sensor_msgs.msg import Image as sensor_msgs_Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.optimize import fsolve
from skimage import morphology
from PIL import Image

# Constants:

IMG_HEIGHT = 720
IMG_WIDTH = 1280

# Topics:
CAMERA_TOPIC = "/zed2/zed_node/right_raw/image_raw_color"
LANE_OVERLAY_TOPIC = "/lane_detection/lane_overlay"
CROSSTRACK_ERROR_TOPIC = "/lane_detection/annotate"
LANE_BIRDSEYE_TOPIC = "lane_detection/birdseye"
LANE_ANNOTATE_TOPIC = "lane_detection/annotate"

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

        # Initialize Publishers
        self.pub_overlay = rospy.Publisher(LANE_OVERLAY_TOPIC, sensor_msgs_Image, queue_size=1)
        self.pub_image = rospy.Publisher(LANE_ANNOTATE_TOPIC, sensor_msgs_Image, queue_size=1)
        self.pub_bird = rospy.Publisher(LANE_BIRDSEYE_TOPIC, sensor_msgs_Image, queue_size=1)

        # Member Variables
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True
        self.copyed = False
    """
    Processes images from Zed2 camera and publishes inference from PyTorch model

    - data: 1 sensor_msgs/Image message from Zed2 Camera
    - return: None
    """ 
    def zed2_image_callback(self, data: sensor_msgs_Image) -> None:
        
        # Convert ROS message to an openCV data type
        cv2_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        
        alpha = 1
        beta = 1
        
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
            transforms.Resize((736, IMG_WIDTH)),  # Resize the image to a valid input size
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
        lane_overlay[ll_seg_mask == 1] = [255, 255, 255]  # Assuming class '1' corresponds to lane lines

        test_var = lane_overlay[4][4]
        # Crop Image back to original resolution:
        lane_overlay = lane_overlay [:IMG_HEIGHT, :]
        
        # lane_overlay = morphology.remove_small_objects(lane_overlay.astype('bool'),min_size=50,connectivity=2)
        
        
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(lane_overlay, 'rgb8'))
        
        # Overlay the binary mask onto the original image
        # Adjust the alpha parameter to control the transparency of the overlay
        # alpha = 0.5
        # img_with_lanes = Image.blend(img_pil, Image.fromarray(lane_overlay), alpha=alpha)
        # print("inference done")
    
    """
    Get bird's eye view from input image. Tunable parameters for pyramid to selec
    """
    def perspective_transform(self, img, verbose=False):
            #1. Visually determine 4 source points and 4 destination points
            #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
            #3. Generate warped image in bird view using cv2.warpPerspective()

            height, width = img.shape[:2]

            # Use a different set of source points for simulator/video
            # Order: upper left, lower left, upper right, lower right
            points1 = np.float32([[500, 406], [330, 563], [715, 406], [880, 563]]) 
            points2 = np.float32([[150,0], [150,600], [650, 0], [650,600]]) 

            M = cv2.getPerspectiveTransform(points1, points2)
            Minv = np.linalg.inv(M)

            warped_img = cv2.warpPerspective(img, M, (800,600))


            warped_img[:, :100] = 0
            warped_img[:, 700:] = 0

            return warped_img, M, Minv    
    
    """
        Takes in a binary image that highlights lanes and outputs

        - img: binary image containing
        - return: None
    """ 
    def detection(self, img):
        img_birdeye, M, Minv = self.perspective_transform(img)

        left_fit, right_fit = None
        
        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False
            
            return self.crossTrackError(left_fit, right_fit)
        
            # # Annotate original image
            # bird_fit_img = None
            # combine_fit_img = None
            # if ret is not None:
            #     bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
            #     combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            # else:
            #     print("Unable to detect lanes")

            # return combine_fit_img, bird_fit_img
    
    
    '''
    Inputs: The x and y coordinates for both of the left and right lane
    Returns: Returns a value that corresponds to how many pixels off of the center line of the lane is compared to the center line 
    of the camera

    Notes: Operates on a set look-ahead distance (1/4 camera height), need to turn right is +, need to turn left is -.


    '''
    def crossTrackError(self, left_fit, right_fit):  

        centerX = IMG_WIDTH / 2
        lookAheadDist = IMG_HEIGHT/4
        minLaneClassify = 10

        

        if len(right_fit)< 10 and len(left_fit) < 10: # If no proper lanes are found
            if(len(right_fit)>len(left_fit)):
                return 100 # assume right is positive
            else:
                return -100
        elif len(right_fit)< 10: # Only left lane is found (turn right more)
            #left_fit = np.polyfit(lefty, leftx, 2)
            leftpoly = np.poly1d(left_fit)
            leftXVal = fsolve(leftpoly - lookAheadDist, centerX)
            return centerX - leftXVal[0] +100 # turn right more


        elif len(left_fit) < 10:
            #right_fit = np.polyfit(righty, rightx, 2)
            rightpoly = np.poly1d(right_fit)
            rightXVal = fsolve(rightpoly - lookAheadDist, centerX)
            return centerX - rightXVal[0] - 100 # turn left more

        else:
            #left_fit = np.polyfit(lefty, leftx, 2)
            #right_fit = np.polyfit(righty, rightx, 2)

            leftpoly = np.poly1d(left_fit)
            rightpoly = np.poly1d(right_fit)

            leftXVal = fsolve(leftpoly - lookAheadDist, centerX)
            rightXVal = fsolve(rightpoly - lookAheadDist, centerX)

            average_Fit = (leftXVal[0] + rightXVal[0]) / 2

            return centerX - average_Fit



if __name__ == '__main__':
    rospy.init_node('lane_det_node')
    LaneDetNode()
    rospy.spin()