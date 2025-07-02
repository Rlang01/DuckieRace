#!/usr/bin/env python3

import rospy
import os
import cv2
import numpy as np
import yaml
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float64
from ultralytics import YOLO
from cv_bridge import CvBridge

class DetectDuckiebotNode(DTROS):
    def __init__(self, node_name):
        super(DetectDuckiebotNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Nutze jetzt separate YAML nur für Duckiebot
        self.config_path = "packages/followlane/config/object_detection_duckiebot.yaml"
        self.load_conf(self.config_path)

        self._model = YOLO("packages/followlane/assets/best_duckiebot.pt")

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size=1)
        self.pub_image = rospy.Publisher(f"/{self._vehicle_name}/detect/duckiebot/image", Image, queue_size=1)
        self.pub_detected = rospy.Publisher(f"/{self._vehicle_name}/detect/duckiebot", Bool, queue_size=1)
        self.pub_distance = rospy.Publisher(f"/{self._vehicle_name}/detect/duckiebot/distance", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.counter = 0
        self.calib_distance = 0.5

        rospy.on_shutdown(self.on_shutdown)

    def load_conf(self, path):
        with open(path, 'r') as f:
            self.obj_conf = yaml.safe_load(f)

        self.focal_length = self.obj_conf["camera"]["focal_length_px"]
        self.frame_skip = self.obj_conf["general"]["frame_skip"]
        self.known_height = self.obj_conf["duckiebot"]["height_m"]
        self.class_id = self.obj_conf["duckiebot"]["class_id"]
        self.min_area = self.obj_conf["duckiebot"]["min_box_area"]

    def cbDetectObjects(self, image_msg):
        if self.counter % self.frame_skip != 0:
            self.counter += 1
            return
        self.counter += 1

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Fehler beim Dekodieren: {e}")
            return

        results = self._model(cv_image)
        image_with_boxes, detected, est_distance = self.process_detections(results, cv_image)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_with_boxes, "bgr8"))
        self.pub_detected.publish(Bool(data=detected))
        self.pub_distance.publish(Float64(data=est_distance))

        cv2.imshow("Duckiebot Detection", image_with_boxes)
        cv2.waitKey(1)

    def process_detections(self, results, img):
        detected = False
        closest_distance = float('inf')

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])

                if class_id != self.class_id:
                    continue

                height_px = y2 - y1
                area = (x2 - x1) * height_px
                if height_px == 0:
                    continue

                distance_m = (self.known_height * self.focal_length) / height_px
                color = (0, 255, 0)

                if area >= self.min_area:
                    detected = True
                    if distance_m < closest_distance:
                        closest_distance = distance_m

                    calibrated_focal = (height_px * self.calib_distance) / self.known_height
                    cv2.putText(img, f"focal_px={calibrated_focal:.1f}", (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    self.obj_conf["camera"]["focal_length_px"] = float(calibrated_focal)
                    with open(self.config_path, 'w') as f:
                        yaml.dump(self.obj_conf, f)
                    rospy.loginfo(f"focal_length_px gespeichert: {calibrated_focal:.1f}")

                cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                cv2.putText(img, f"Duckiebot {distance_m:.2f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return img, detected, (closest_distance if detected else -1.0)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("DetectDuckiebotNode wurde sauber beendet.")

if __name__ == '__main__':
    node = DetectDuckiebotNode(node_name='detect_duckiebot_node')
    rospy.spin()
