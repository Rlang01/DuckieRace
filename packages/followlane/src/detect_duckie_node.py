#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO
from std_msgs.msg import Float64
from cv_bridge import CvBridge


class DetectDuckieNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectDuckieNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # YOLO Modell laden
        self._model = YOLO("packages/followlane/assets/model.pt")

        # Fahrzeugname aus Umgebungsvariablen
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Subscriber
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size=1)

        # Publisher
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckie/image"
        self.pup_image = rospy.Publisher(self._yolo_topic, Image, queue_size=1)
        self._duckie_topic = f"/{self._vehicle_name}/detect/duckie"
        self.pup_duckie = rospy.Publisher(self._duckie_topic, Float64, queue_size=1)

        self.counter = 0
        self.bridge = CvBridge()

    def cbDetectObjects(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        self.counter += 1

        # CompressedImage in OpenCV-Image umwandeln
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # YOLO Inferenz
        results = self._model(cv_image)

        # Bounding Boxes zeichnen
        image_with_boxes = draw_bounding_boxes(results, cv_image)

        # Bild veröffentlichen (ROS)
        msg = self.bridge.cv2_to_imgmsg(image_with_boxes, "bgr8")
        self.pup_image.publish(msg)

        # Bild auch lokal anzeigen
        cv2.imshow("Duckiebot-Detection", image_with_boxes)
        cv2.waitKey(1)

def draw_bounding_boxes(results, img):
    for result in results:
        for box in result.boxes:
            cv2.rectangle(img,
                          (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])),
                          (255, 0, 0), 5)
            cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
    return img

if __name__ == '__main__':
    node = DetectDuckieNode(node_name='detect_duckie_node')
    rospy.spin()
