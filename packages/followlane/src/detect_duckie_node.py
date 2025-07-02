#!/usr/bin/env python3

import rospy
import os
import cv2
import yaml
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float64
from ultralytics import YOLO
from cv_bridge import CvBridge

class DetectDuckieNode(DTROS):
    def __init__(self, node_name):
        super(DetectDuckieNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Nutze separate YAML nur für Duckie
        self.config_path = "packages/followlane/config/object_detection_duckie.yaml"
        self.load_conf(self.config_path)

        # Lade YOLO-Modell für Duckie
        self._model = YOLO("packages/followlane/assets/best_duckie.pt")

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size=1)
        self.pub_image = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie/image", Image, queue_size=1)
        self.pub_detected = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie", Bool, queue_size=1)
        self.pub_distance = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie/distance", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.counter = 0

        rospy.on_shutdown(self.on_shutdown)

    def load_conf(self, path):
        with open(path, 'r') as f:
            conf = yaml.safe_load(f)
        self.focal_length = conf["camera"]["focal_length_px"]
        self.frame_skip = conf["general"]["frame_skip"]
        self.known_height = conf["duckie"]["height_m"]
        self.class_id = conf["duckie"]["class_id"]
        self.min_area = conf["duckie"]["min_box_area"]

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
        image_with_boxes, duckie_near, estimated_distance = self.process_detections(results, cv_image)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_with_boxes, "bgr8"))
        self.pub_detected.publish(Bool(data=duckie_near))
        self.pub_distance.publish(Float64(data=estimated_distance))

        cv2.imshow("Duckie Detection", image_with_boxes)
        cv2.waitKey(1)

    def process_detections(self, results, img):
        duckie_near = False
        closest_distance = float('inf')

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                if class_id != self.class_id:
                    continue
                height_px = y2 - y1
                area = (x2 - x1) * height_px
                distance_m = (self.known_height * self.focal_length) / height_px if height_px > 0 else float('inf')
                color = (0, 255, 255)

                if area >= self.min_area:
                    duckie_near = True
                    if distance_m < closest_distance:
                        closest_distance = distance_m

                cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                cv2.putText(img, f"Duckie {distance_m:.2f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return img, duckie_near, (closest_distance if duckie_near else -1.0)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("DetectDuckieNode wurde sauber beendet.")

if __name__ == '__main__':
    node = DetectDuckieNode(node_name='detect_duckie_node')
    rospy.spin()
