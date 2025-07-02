#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Int32
from enum import Enum
import os
import time
from duckietown.dtros import DTROS, NodeType

# === Definition der möglichen Steuerungsmodi ===
class ControlType(Enum):
    followLane = 1          # Normaler Spurfolgemodus
    wait = 2                # (Optional, hier ungenutzt)
    bypassObstacle = 3      # Ausweichmodus bei Hindernis

# === Hauptklasse des Nodes ===
class SwitchControlNode(DTROS):
    def __init__(self, node_name):
        # Initialisierung des ROS-Nodes mit Duckietown-spezifischem Typ
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Fahrzeugname aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Interne Zustände
        self._control_mode = ControlType.followLane      # Startmodus: normal folgen
        self._last_duckie_seen_time = None               # Zeitpunkt der letzten Duckie-Erkennung
        self._duckie_close = False                       # Aktueller Duckie-Status (erkannt oder nicht)
        self._direction = 1                              # Richtung des Spurwechsels: +1 = links, -1 = rechts

        # === Publisher ===
        self.pub_control = rospy.Publisher(
            f"/{self._vehicle_name}/switch/control", Int32, queue_size=1
        )
        self.pub_direction = rospy.Publisher(
            f"/{self._vehicle_name}/switch/direction", Int32, queue_size=1
        )

        # === Subscriber ===
        # Gibt an, ob ein Duckie erkannt wurde (True/False)
        self.sub_duckie = rospy.Subscriber(
            f"/{self._vehicle_name}/detect/duckie", Bool, self.cb_duckie_detected, queue_size=1
        )
        # Gibt den Abstand zum nächsten Duckie in Metern
        self.sub_distance = rospy.Subscriber(
            f"/{self._vehicle_name}/detect/duckie/distance", Float64, self.cb_duckie_distance, queue_size=1
        )

        rospy.loginfo("SwitchControlNode erfolgreich gestartet.")

    # === Callback: Duckie erkannt oder nicht ===
    def cb_duckie_detected(self, msg: Bool):
        if msg.data:
            # Duckie wurde erkannt → Zeitstempel speichern
            self._last_duckie_seen_time = time.time()
        # Merke aktuellen Duckie-Zustand
        self._duckie_close = msg.data

    # === Callback: Abstand zum Duckie ===
    def cb_duckie_distance(self, msg: Float64):
        current_time = time.time()
        distance = msg.data

        # --- Auslöser 1: Duckie < 30cm entfernt → Nach links ausweichen ---
        if 0 < distance < 0.3 and self._control_mode == ControlType.followLane:
            rospy.loginfo("Duckie erkannt bei <30cm → starte Linkskurve.")
            self._control_mode = ControlType.bypassObstacle
            self._direction = 1  # Nach links ausweichen
            return

        # --- Auslöser 2: Seit >1s kein Duckie mehr gesehen → zurück nach rechts ---
        if (self._control_mode == ControlType.followLane and
            self._last_duckie_seen_time is not None and
            (current_time - self._last_duckie_seen_time) > 1.0):
            rospy.loginfo("Seit >1s kein Duckie erkannt → starte Rechtskurve zurück.")
            self._control_mode = ControlType.bypassObstacle
            self._direction = -1  # Zurück nach rechts

    # === Haupt-Loop ===
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Aktuellen Steuerungsmodus veröffentlichen
            self.pub_control.publish(Int32(self._control_mode.value))
            self.pub_direction.publish(Int32(self._direction))

            # Sobald der Bot im Ausweichmodus ist und keine Duckies mehr sieht → zurück zu followLane
            if self._control_mode == ControlType.bypassObstacle and not self._duckie_close:
                self._control_mode = ControlType.followLane

            rate.sleep()

# === Main ===
if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()
