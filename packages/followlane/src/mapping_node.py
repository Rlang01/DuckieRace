#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml


from duckietown.dtros import DTROS, NodeType

class MappingNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MappingNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._bridge = CvBridge()
        self.load_conf('packages/followlane/config/mapping.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.pub_map_present = rospy.Publisher(f'/{self._vehicle_name}/mapping/present', CompressedImage, queue_size = 1)

        self.counter = 0

    def calc_transform_matrix(self, img):
        rows,cols,ch = img.shape
        self.x_output = cols

        pts_orig_target = [[0,0],[self.x_output,0],[0,self.x_output],[self.x_output,self.x_output]]

        # crop the input image
        cutoff_pts = [(0,int(rows*self.top_cutoff)),(cols,int(rows*self.top_cutoff))]
        self.cutoff_y = cutoff_pts[0][1]
        pts_orig_cropped = [[x, y-cutoff_pts[0][1]] for x, y in self.pt_orig]

        # calculate the initial transformation matrix
        M = cv2.getPerspectiveTransform(np.float32(pts_orig_cropped),np.float32(pts_orig_target))

        # calculate transformed corners
        corners = [[0,0],[cols,0],[0,rows],[cols,rows]]
        corners_transformed = []
        for corner in corners:
            corner = np.array([corner[0], corner[1], 1])
            corner = corner.reshape(3, 1)
            corner_transformed = np.dot(M,corner)
            corner_transformed = corner_transformed/corner_transformed[2]
            corners_transformed.append((int(corner_transformed[0]), int(corner_transformed[1])))

        # calculate needed scaling & translation and new size
        new_x_size = abs(corners_transformed[1][0] - corners_transformed[0][0])
        new_y_size = abs(corners_transformed[2][1] - corners_transformed[0][1])
        tx = - corners_transformed[0][0] # x-translation
        ty = - corners_transformed[0][1] # y-translation
        fx = new_x_size/self.x_output # scaling
        self.y_output = int(new_y_size/fx) # y size of output image

        # calculate new target coordinates (for calibration object)
        pts_adj_target = [[int((x+tx)/fx), int((y+ty)/fx)] for x, y in pts_orig_target]

        # calculate new transformation matrix
        self.M_transform = cv2.getPerspectiveTransform(np.float32(pts_orig_cropped),np.float32(pts_adj_target))


    def cbMakePresentMap(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1


        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = cv_image.copy()
        rows,cols,ch = img.shape

        if not self.M_transform:
            self.calc_transform_matrix(img)

        img_crop = img[self.cutoff_y:rows,0:cols]
        dst = cv2.warpPerspective(img_crop,self.M_transform,(self.x_output,self.y_output))
        # TODO: calculate measurements (or transformed measurements)


        # output map here. TODO: find good data format
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode(".jpg", dst)[1]).tostring()

        self.pub_map_present.publish(msg)

    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.top_cutoff = self.conf['params']['topLimit']
        self.pt_orig = [[self.conf['calibration']['chess_pt1_x'],self.conf['calibration']['chess_pt1_y']],
                        [self.conf['calibration']['chess_pt2_x'],self.conf['calibration']['chess_pt2_y']],
                        [self.conf['calibration']['chess_pt3_x'],self.conf['calibration']['chess_pt3_y']],
                        [self.conf['calibration']['chess_pt4_x'],self.conf['calibration']['chess_pt4_y']]]
            
        
if __name__ == '__main__':

    node = MappingNode(node_name='mapping_node')
    rospy.spin()
