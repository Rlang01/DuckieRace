#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml


from duckietown.dtros import DTROS, NodeType

class MappingNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MappingNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)


        self.load_conf('packages/followlane/config/mapping.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.pub_map_present = rospy.Publisher(f'/{self._vehicle_name}/mapping/present', Float64, queue_size = 1)

        self.counter = 0

    def crop_img(self,img):
        img = img.copy()
        print(img.shape)

        pts1 = np.float32([
            [self.conf['lane_image']['top_left_x'],     self.conf['lane_image']['top_left_y']],
            [self.conf['lane_image']['top_right_x'],    self.conf['lane_image']['top_right_y']],
            [self.conf['lane_image']['bottom_right_x'], self.conf['lane_image']['bottom_right_y']],
            [self.conf['lane_image']['bottom_left_x'],  self.conf['lane_image']['bottom_left_y']],])
        
        pts2 = np.float32([[0,0],[100,0],[0,100],[100,100]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(100,100))

    def cbMakePresentMap(self, image_msg):
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

        # image maximal sizes 
        y_max,x_max,ch = img.shape
        y_limit = round(y_max * self.top_cutoff, 0)
        x_mid = round(x_max / 2, 0)
        dist = 100 # distance between two reference points in pixel

        # set anchor points (2 at bottom corners, 2 on horizontal cutoff line and with calibrated distance)
        pts1 = [[x_mid-dist/2,y_limit],[x_mid+dist/2,y_limit],[0,y_max],[x_max,y_max]] # from top left in Z-shape
        pts2 = [[0,0],[x_max,0],[0,y_max],[x_max,y_max]]

        # TODO: check if two Transform matrices can be merged so only one transformation is needed
        M = cv2.getPerspectiveTransform(np.float32(pts1),np.float32(pts2))
        img_flat = cv2.warpPerspective(img,M,(x_max,y_max))

        # TODO: add padding at bottom for the distance of vehicle reference point to bottom line of image


        # output map here. TODO: find good data format
        #msg_desired_center = 
        #msg_map.data = 
        #self.mapping.publish(msg_map)

    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.camera_angle_degr = self.conf['camera']['angle']
        self.camera_height = self.conf['camera']['height']
        self.top_cutoff = self.conf['params']['topLimit']
        self.d1 = self.conf['calibration']['d1']
        self.d2 = self.conf['calibration']['d2']
        self.d3 = self.conf['calibration']['d3']
            
        
if __name__ == '__main__':

    node = MappingNode(node_name='mapping_node')
    rospy.spin()
