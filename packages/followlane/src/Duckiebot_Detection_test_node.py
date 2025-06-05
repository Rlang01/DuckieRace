#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO
from cv_bridge import CvBridge


class DetectDuckiebotNode(DTROS):
    def __init__(self, node_name):
        super(DetectDuckiebotNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # YOLO Modell laden
        self._model = YOLO("packages/followlane/assets/model.pt")  

        # Fahrzeugname aus Umgebungsvariablen
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Subscriber
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size=1)

        # Publisher
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckiebot/image"
        self.pub_image = rospy.Publisher(self._yolo_topic, Image, queue_size=1)

        self.bridge = CvBridge()
        self.counter = 0

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
        image_with_boxes = self.draw_bounding_boxes(results, cv_image)

        # Bild zurück in ROS-Image konvertieren und publishen
        msg = self.bridge.cv2_to_imgmsg(image_with_boxes, "bgr8")
        self.pub_image.publish(msg)
        cv2.imshow("Duckiebotaugemacher", image_with_boxes )

    def draw_bounding_boxes(self, results, img):
        for result in results:
            for box in result.boxes:
                # Koordinaten abrufen
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                conf = box.conf[0]

                # Nur Duckiebots anzeigen (Klasse 0)
                if class_id == 0:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(img, f"Duckiebot {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return img


if __name__ == '__main__':
    node = DetectDuckiebotNode(node_name='detect_duckiebot_node')
    rospy.spin()
