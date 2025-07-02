#!/usr/bin/env python3

import rospy
import os
import numpy as np
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Pose2D
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType

class ControlObstacleNode(DTROS):
    def __init__(self, node_name):
        super(ControlObstacleNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # === Parameter ===
        self.v = rospy.get_param("~forward_speed", 0.3)             # Vorwärtsgeschwindigkeit in m/s
        self.omega = rospy.get_param("~omega", 0.6)                 # Drehgeschwindigkeit in rad/s
        self.lane_width = rospy.get_param("~lane_width", 0.18)      # Breite einer Spur (Standard: 18cm)
        self.direction = rospy.get_param("~direction", 1)           # +1 = links, -1 = rechts
        self.lateral_shift = rospy.get_param("~lateral_shift", self.lane_width)  # Zielversatz zur Seite

        # === Radius der Kreisbahn (r = v / omega) ===
        self.radius = self.v / self.omega

        # === Überprüfung: lateral_shift < 2 * radius, sonst ist acos undefiniert ===
        # Beispiel: v=0.3, omega=0.6 → radius = 0.5 m → maximaler gültiger Versatz ≈ 1.0 m
        try:
            self.turn_distance = self.radius * np.arccos(1 - self.lateral_shift / self.radius)
        except ValueError:
            rospy.logerr("Ungültige Geometrie: lateral_shift zu groß für diesen Radius.")
            self.turn_distance = 1.0  # Fallback-Wert

        # === Gesamte Strecke für Hin- und Rückkurve ===
        self.total_distance = 2 * self.turn_distance

        # === Interner Zustand ===
        self.s_current = 0.0                     # Aktuelle Strecke seit Start des Ausweichens
        self.phase = 0                           # 0 = hinlenken, 1 = zurücklenken
        self.avoiding = False                    # Aktiver Ausweichstatus
        self.pose_current = Pose2D()             # Aktuelle Pose (aus Odometrie)
        self.pose_start = None                   # Startposition bei Beginn des Ausweichmanövers

        # === Publisher ===
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.pub_done = rospy.Publisher(f"/{self._vehicle_name}/obstacle_avoidance/done", Bool, queue_size=1)

        # === Subscriber ===
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control, queue_size=1)
        self.sub_pose = rospy.Subscriber(f"/{self._vehicle_name}/odometry/pose", Pose2D, self.cb_pose, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("ControlObstacleNode (mit echter Odometrie) gestartet.")

    # === Callback: Steuerbefehl empfangen ===
    def cb_control(self, msg):
        if msg.data == ControlType.bypassObstacle.value and not self.avoiding:
            rospy.loginfo(f"Starte Ausweichmanöver ({self.lateral_shift:.2f} m seitlich).")
            self.avoiding = True
            self.pose_start = self.pose_current  # Startposition speichern
            self.phase = 0
            self.publish_done(False)

    # === Callback: aktuelle Odometriepose speichern ===
    def cb_pose(self, msg: Pose2D):
        self.pose_current = msg

    # === Berechnung der seit Start gefahrenen Strecke (euklidisch) ===
    def compute_distance(self):
        dx = self.pose_current.x - self.pose_start.x
        dy = self.pose_current.y - self.pose_start.y
        return np.sqrt(dx**2 + dy**2)

    # === Hauptloop ===
    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if self.avoiding and self.pose_start:
                self.s_current = self.compute_distance()  # Strecke per Odometrie bestimmen
                self.execute_step()
            rate.sleep()

    # === Führt einen Schritt des Ausweichmanövers aus ===
    def execute_step(self):
        if self.s_current >= self.total_distance:
            rospy.loginfo("Ausweichmanöver abgeschlossen (Odometrie-basiert).")
            self.avoiding = False
            self.send_stop()
            self.publish_done(True)
            return

        # Phase: Hin- oder Rückkurve
        if self.s_current < self.turn_distance:
            omega = self.direction * self.omega
        else:
            omega = -self.direction * self.omega

        # Fahre mit festem v und omega
        twist = Twist2DStamped()
        twist.v = self.v
        twist.omega = omega
        self.pub_cmd_vel.publish(twist)

    # === Stoppe den Bot ===
    def send_stop(self):
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    # === Signalisiere Abschluss des Manövers ===
    def publish_done(self, done):
        self.pub_done.publish(Bool(data=done))

    # === Bei Shutdown: stoppe sicher ===
    def on_shutdown(self):
        rospy.loginfo("ControlObstacleNode wird beendet.")
        self.send_stop()
        self.publish_done(True)

if __name__ == '__main__':
    node = ControlObstacleNode(node_name='control_obstacle_node')
    node.run()
