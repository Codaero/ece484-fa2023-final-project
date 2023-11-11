import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology

# CHANGE THIS LINE TO SWITCH BETWEEN GEM GAZEBO AND REAL LIFE
GEM_SIMULATOR = False
ROSBAG_3 = True

class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        assert GEM_SIMULATOR != ROSBAG_3 # impossible

        if GEM_SIMULATOR:
            self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback_sim, queue_size=1)
        elif ROSBAG_3:
            self.sub_image = rospy.Subscriber('zed2/zed_node/rgb/image_rect_color', Image, self.img_callback_real, queue_size=1)
        else:
            self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback_real, queue_size=1)

        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True
        self.copyed = False

    def img_callback_real(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()

        if not self.copyed:
            cv2.imwrite("video.png", raw_img)
            self.copyed = not self.copyed

        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def color_thresh_video(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass

        # 1 - Convert image from RGB to HSL
        hsl_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # 2 - Threshold L channel for white-looking lines
        white_filter = cv2.inRange(hsl_img[:, :, 1], 215, 255)

        if ROSBAG_3:
            yellow_low = np.array([25, 0, 90])
            yellow_high = np.array([35, 255, 255])
            white_filter1 = cv2.inRange(hsl_img[:, :, 1], 150, 255)
            white_filter = white_filter1

            yellow_filter = cv2.inRange(hsl_img, yellow_low, yellow_high)
            yw_mask = cv2.bitwise_or(yellow_filter, white_filter) / 255 # Convert 0-255 to 0-1
            
            dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            yw_mask = cv2.dilate(yw_mask, dilate_element)
            return yw_mask

        # 3 - Dilate area a little to coincide with sobel output later down the pipeline
        yw_mask = white_filter / 255 # Convert 0-255 to 0-1
        
        dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        yw_mask = cv2.dilate(yw_mask, dilate_element)

        ####

        return yw_mask

    def color_thresh(self, img, thresh=(100, 255)):
        # Select which method to use 
        if GEM_SIMULATOR:
            return self.color_thresh_gem(img)
        else:
            return self.color_thresh_video(img)

    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        SobelOutput = self.gradient_thresh(img)
        ColorOutput = self.color_thresh(img)

        binaryImage = np.zeros_like(SobelOutput)

        if GEM_SIMULATOR:
            binaryImage = ColorOutput
        else:
            if ROSBAG_3:
                binaryImage[(ColorOutput==1) | (SobelOutput==1)] = 1
            else:
                binaryImage[(ColorOutput==1) & (SobelOutput==1)] = 1

        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        binaryImage = binaryImage.astype(np.uint8)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        height, width = img.shape[:2]

        # Use a different set of source points for simulator/video
        # Order: upper left, lower left, upper right, lower right
        if GEM_SIMULATOR:
            points1 = np.float32([ [0.39* width, 60*height/100], [0, height-10]  , [0.61*width, 60*height/100], [width, height-10]])
            points2 = np.float32([[150,0], [150,600], [650, 0], [650,600]])
        else:
            points1 = np.float32([[500, 406], [330, 563], [715, 406], [880, 563]]) if ROSBAG_3 else np.float32([[464, 256], [266, 370], [726, 254], [800, 339]])
            points2 = np.float32([[150,0], [150,600], [650, 0], [650,600]]) if ROSBAG_3 else np.float32([[100,0], [100,600], [700, 0], [700,600]])

        M = cv2.getPerspectiveTransform(points1, points2)
        Minv = np.linalg.inv(M)

        warped_img = cv2.warpPerspective(img, M, (800,600))

        if GEM_SIMULATOR:
            pass
        else:
            warped_img[:, :100] = 0
            warped_img[:, 700:] = 0
        
        ####

        return warped_img, M, Minv


    def detection(self, img):
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

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

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
