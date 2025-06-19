#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
from scipy.stats import linregress


from duckietown.dtros import DTROS, NodeType

class DetectLaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectLaneNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)


        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        self.counter = 0

    def crop_img(self,img):
        img = img.copy()

        pts1 = np.float32([
            [self.conf['lane_image']['top_left_x'],     self.conf['lane_image']['top_left_y']],
            [self.conf['lane_image']['top_right_x'],    self.conf['lane_image']['top_right_y']],
            [self.conf['lane_image']['bottom_right_x'], self.conf['lane_image']['bottom_right_y']],
            [self.conf['lane_image']['bottom_left_x'],  self.conf['lane_image']['bottom_left_y']],])
        
        pts2 = np.float32([[0,0],[100,0],[0,100],[100,100]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(100,100))

    def cbFindLane(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1

        # Write your own Code for Lane detection here
        # This is only a basic example to get some inspiration from

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)

        try:
            coords = cv2.findNonZero(mask_yellow) #get the x/y values of the masked pixels
            x, y = coords.T
            yellow_slope, yellow_intercept, yellow_r_value, _, _ = linregress(x, y) # find regression line
            coords = cv2.findNonZero(mask_white) #get the x/y values of the masked pixels
            x, y = coords.T
            white_slope, white_intercept, white_r_value, _, _ = linregress(x, y) # find regression line

            target_x = ((- white_intercept / white_slope) - (yellow_intercept / yellow_slope)) / 2
            error = img.shape[1] - target_x
            print(f"error: {round(error,2)}, white r^2: {round(white_r_value ** 2,2)}, yellow r^2: {round(yellow_r_value ** 2,)}")
            view_data_y = [yellow_slope, yellow_intercept, yellow_r_value]
            view_data_w = [white_slope, white_intercept, white_r_value]
        except:
            error = 0
            print("error in lane detection")
            view_data_y = [0,0,0]
            view_data_w = [0,0,0]


        msg_desired_center = Float64()
        msg_desired_center.data = error
        self.pub_lane.publish(msg_desired_center)

        # combine both masks
        img_view = cv2.bitwise_or(mask_white, mask_yellow)

        img_view = self.show_view(img_view, view_data_y, view_data_w)
        cv2.imshow("Lane Detection", img_view)
        cv2.waitKey(1)


    def show_view(self, img, yellow_data=[0,0,0], white_data=[0,0,0]):
        # draw a yellow line on th image
        cv2.line(img, (0, int(yellow_data[1])), (img.shape[1], int(img.shape[1] * yellow_data[0] + yellow_data[1])), (255, 222, 89), 2)
        cv2.putText(img, f"Y: {round(yellow_data[2] ** 2, 2)}", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 222, 89), 2)
        # draw a yellow line on th image
        cv2.line(img, (0, int(white_data[1])), (img.shape[1], int(img.shape[1] * white_data[0] + white_data[1])), (255, 0, 0), 2)
        cv2.putText(img, f"W: {round(white_data[2] ** 2, 2)}", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)

        return img
        

    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.hue_white_l = self.conf['white']['hl']
        self.hue_white_h = self.conf['white']['hh']
        self.saturation_white_l = self.conf['white']['sl']
        self.saturation_white_h = self.conf['white']['sh']
        self.lightness_white_l = self.conf['white']['vl']
        self.lightness_white_h = self.conf['white']['vh']
        
        self.hue_yellow_l = self.conf['yellow']['hl']
        self.hue_yellow_h = self.conf['yellow']['hh']
        self.saturation_yellow_l =  self.conf['yellow']['sl']
        self.saturation_yellow_h =  self.conf['yellow']['sh']
        self.lightness_yellow_l =  self.conf['yellow']['vl']
        self.lightness_yellow_h =  self.conf['yellow']['vh']
        
        self.hue_duck_l = self.conf['duck']['hl']
        self.hue_duck_h = self.conf['duck']['hh']
        self.saturation_duck_l =  self.conf['duck']['sl']
        self.saturation_duck_h =  self.conf['duck']['sh']
        self.lightness_duck_l =  self.conf['duck']['vl']
        self.lightness_duck_h =  self.conf['duck']['vh']
            
        
if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
