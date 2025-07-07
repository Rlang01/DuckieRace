#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import yaml
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class RedLineDistanceNode:
    def __init__(self):
        rospy.init_node("red_line_distance_node", anonymous=False)

        # Pfad zur Konfigurationsdatei laden
        self.config_path = "packages/followlane/config/detect_lane.yaml"
        self.load_red_thresholds(self.config_path)

        self.bridge = CvBridge()
        self.latest_image = None
        self.frame_counter = 0
        self.frame_skip = 3  # nur jedes dritte Bild verarbeiten

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.image_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.pub_distance = rospy.Publisher(f"/{self.vehicle_name}/red_line_distance", Float32, queue_size=1)
        self.sub_image = rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RedLineDistanceNode läuft.")
        rospy.spin()

    def load_red_thresholds(self, path):
        with open(path, 'r') as f:
            conf = yaml.safe_load(f)
        red = conf['red']
        self.red_lower1 = np.array([red['hl'], red['sl'], red['vl']])
        self.red_upper1 = np.array([red['hh'], red['sh'], red['vh']])
        # zweite Rot-Maske für H > 160 (falls nötig)
        self.red_lower2 = np.array([160, red['sl'], red['vl']])
        self.red_upper2 = np.array([179, red['sh'], red['vh']])

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Fehler beim Dekodieren des Bildes: {e}")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # zwei Rotbereiche maskieren
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Finden der Position der untersten (tiefsten) roten Pixel
        red_pixels = cv2.findNonZero(mask)

        if red_pixels is not None:
            # größter y-Wert → unterster Punkt im Bild
            max_y = max([pt[0][1] for pt in red_pixels])
            height = img.shape[0]
            y_norm = max_y / height

            # einfache lineare Näherung für Distanz
            estimated_distance = round(0.5 * (1.0 - y_norm), 2)

            rospy.loginfo(f"Rote Linie erkannt, Distanz: {estimated_distance:.2f} m")
            self.pub_distance.publish(Float32(estimated_distance))
            cv2.putText(img, f"Distanz: {estimated_distance:.2f} m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            rospy.loginfo("Keine rote Linie erkannt.")
            self.pub_distance.publish(Float32(-1.0))
            cv2.putText(img, "Keine rote Linie erkannt", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # zeige Bild am Entwickler-Rechner
        cv2.imshow("Red Line Detection", img)
        cv2.waitKey(1)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("RedLineDistanceNode wurde beendet.")


if __name__ == "__main__":
    try:
        RedLineDistanceNode()
    except rospy.ROSInterruptException:
        pass
