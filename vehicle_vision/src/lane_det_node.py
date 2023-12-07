#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math

import torch
from torchvision import transforms

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line

from sensor_msgs.msg import Image as sensor_msgs_Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.optimize import fsolve
from skimage import morphology
from PIL import Image
from std_msgs.msg import Float32


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
        self.pub_lane_orientation = rospy.Publisher('/lane_orientation', Float32, queue_size=1)

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
        cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # alpha = 1
        # beta = 1
        
        # cv2_img = cv2.convertScaleAbs(cv2_img, alpha=alpha, beta=beta)
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


        # Preprocess the image to filter out the yellow parking lot
#         hsl_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HLS)

#         yellow_low = np.array([25, 0, 90])
#         yellow_high = np.array([35, 255, 255])

#         yellow_filter = cv2.inRange(hsl_img, yellow_low, yellow_high)
        
#         rgba_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2BGRA)
# # Set the alpha channel to zero (transparent) wherever yellow is detected
#         rgba_img[:, :, 3] = np.where(yellow_filter, 0, 255)
        
#         cv2.imshow("rgba", rgba_img)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
        
#         cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_HLS2RGB)
        

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
        
        # Convert to binary mask:
        lane_overlay[ll_seg_mask == 1] = [255,255,255]  # Assuming class '1' corresponds to lane lines

        # Crop Image back to original resolution:
        lane_overlay = lane_overlay [:IMG_HEIGHT, :]
        
        # lane_overlay = morphology.remove_small_objects(lane_overlay.astype('bool'),min_size=50,connectivity=2)
        # lane_overlay = lane_overlay.astype(np.uint8)
        
        kernel = (9, 9)
        lane_overlay = cv2.morphologyEx(lane_overlay, cv2.MORPH_OPEN, kernel)
        
        warped_img, M, Minv = self.perspective_transform(lane_overlay)
        
        binary_lane_overlay = warped_img[:, :, 0] / 255
        binary_lane_overlay = np.array(binary_lane_overlay, dtype=int)
        
        self.detection(binary_lane_overlay)
        
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(lane_overlay, 'rgb8'))

        self.pub_bird.publish(self.bridge.cv2_to_imgmsg(warped_img, 'rgb8'))
        
        # Overlay the binary mask onto the original image
        # Adjust the alpha parameter to control the transparency of the overlay
        # alpha = 0.5
        # img_with_lanes = Image.blend(img_pil, Image.fromarray(lane_overlay), alpha=alpha)
        # print("inference done")
    
    """
    Get bird's eye view from input image. Tunable parameters for pyramid to select.
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
        
        
        points = np.array([[(300, 660), (550,425), (800,425), (1050, 660)]]) 
        masked_im = np.zeros_like(img)
        cv2.fillPoly(masked_im, points, color=(255, 255, 255))
        
        combined_image = cv2.bitwise_and(img, masked_im, mask=None)

        # Copy edges to the images that will display the results in BGR
        cdstP = cv2.cvtColor(combined_image, cv2.COLOR_GRAY2BGR)
        # linesP = cv2.HoughLinesP(combined_image, cv2.HOUGH_PROBABILISTIC, np.pi / 180, 150, minLineLength=50, maxLineGap=50)
        linesP = cv2.HoughLinesP(combined_image, cv2.HOUGH_PROBABILISTIC, np.pi / 180, 50, minLineLength=70, maxLineGap=50)
        if linesP is None: print(linesP) 

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        lane_overlay = cv2.addWeighted(img,1,cdstP,1,1)
        # print(lane_overlay.shape)

        #Bird's eye view for Gem
        tl = (500, 425)
        bl = (280, 660)
        tr = (780, 425)
        br = (1000, 660)
        source_pnts = np.float32([tl, bl, tr, br])
        dest_pnts = np.float32([[0,0], [0,720], [1280,0], [1280,720]])
        # for i in range(0,4):
        #         cv2.circle(lane_overlay,(source_pnts[i][0], source_pnts[i][1]),5,(0,0,255),2)

        M = cv2.getPerspectiveTransform(source_pnts, dest_pnts)
        birds_eye = cv2.warpPerspective(cdstP, M, (img.shape[1], img.shape[0]))
        
        # plt.imshow(birds_eye)
        # plt.show()
        gray_birdseye = cv2.cvtColor(birds_eye, cv2.COLOR_BGR2GRAY)
        gray_birdseye = cv2.GaussianBlur(gray_birdseye, (3,3), 0)
        gray_birdseye = cv2.Canny(gray_birdseye, 50, 95, None, 3)
        lines_birdseye = cv2.HoughLinesP(gray_birdseye, cv2.HOUGH_PROBABILISTIC, np.pi / 180, 80, minLineLength=70, maxLineGap=50)

        if lines_birdseye is not None:
            for l in lines_birdseye:
                x1, y1, x2, y2 = l[0]
                #left lane
                if x1 < 640:
                    x1_left = x1
                    x2_left = x2
                    y1_left = y1
                    y2_left = y2
                
                # elif x1 > 640 or x2 > 640:
                #     x1_right = x1
                #     x2_right = x2
                #     y1_right = y1
                #     y2_right = y2
                
                try:
                    # calculate middle points
                    # x1_mid = int((x1_right + x1_left)/2)
                    # x2_mid = int((x2_right + x2_left)/2)
                
                    # y1_mid = int((y1_right + y1_left)/2)
                    # y2_mid = int((y2_right + y2_left)/2)
                
                    # cv2.line(birds_eye, (640, 300), (x2_mid, 420), (0, 255, 0), 2)
                    
                    
                    #draw center line in the middle of the birds eye 
                    xc_1, xc_2 = 640, 640
                    yc_1, yc_2 = 720, 420
                    cv2.line(birds_eye, (xc_1,yc_1), (xc_2, yc_2), (0, 0, 255), 2)
                
                    #calculate angle between line and center line
                    if x2_left > x1_left and y2_left > y1_left:
                        ang_rad = np.arctan2(x2_left-x1_left, y2_left-y1_left) 
                        angle = (ang_rad *180 / np.pi)

                        self.pub_lane_orientation.publish(math.radians(angle))

                        x_dist = int(200*math.cos(ang_rad))
                        cv2.line(birds_eye, (xc_1,yc_1), (xc_2+x_dist, yc_2), (255, 0, 255), 2)

                except NameError:
                    continue
                cv2.line(birds_eye, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # self.pub_steering_angle.publish(angle)

        return lane_overlay, birds_eye

    
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

        

        if len(right_fit)< minLaneClassify and len(left_fit) < minLaneClassify: # If no proper lanes are found
            if(len(right_fit)>len(left_fit)):
                return 100 # assume right is positive
            else:
                return -100
        elif len(right_fit)< minLaneClassify: # Only left lane is found (turn right more)
            #left_fit = np.polyfit(lefty, leftx, 2)
            leftpoly = np.poly1d(left_fit)
            leftXVal = fsolve(leftpoly - lookAheadDist, centerX)
            return centerX - leftXVal[0] +100 # turn right more


        elif len(left_fit) < minLaneClassify:
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