#!/usr/bin/env python3

import rospy
import os
import math
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from duckietown.dtros import DTROS, NodeType


class OdometryNode(DTROS):
    def __init__(self, node_name):
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # === Publisher ===
        self.pub_pose = rospy.Publisher(f"/{self._vehicle_name}/odometry/pose", Pose2D, queue_size=1)

        # === Subscriber ===
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control)
        self.sub_imu = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/data", Imu, self.cb_imu)
        self.sub_temperature = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/temperature", Temperature, self.cb_temp)

        # === Interner Zustand ===
        self.x = 0.0          # Position in x-Richtung (vor/zurück)
        self.y = 0.0          # Position in y-Richtung (seitlich)
        self.theta = 0.0      # Orientierung (Drehwinkel)
        self.vx = 0.0         # Geschwindigkeit in x-Richtung (m/s)
        self.vy = 0.0         # Geschwindigkeit in y-Richtung (m/s)
        self.last_time = None

    # === Callback: Steuermodus anzeigen ===
    def cb_control(self, msg: Int32):
        rospy.loginfo(f"Steuermodus: {msg.data}")

    # === Callback: Temperaturdaten ===
    def cb_temp(self, msg: Temperature):
        rospy.loginfo(f"Temperatur: {msg.temperature:.2f} °C")

    # === Callback: IMU-Daten auswerten und Pose berechnen ===
    def cb_imu(self, msg: Imu):
        current_time = msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # === Sensorwerte ===
        ax = msg.linear_acceleration.x     # Beschleunigung nach vorne (m/s²)
        ay = msg.linear_acceleration.y     # Beschleunigung seitlich (m/s²)
        wz = msg.angular_velocity.z        # Drehgeschwindigkeit um z-Achse (rad/s)

        # === Integriere Geschwindigkeit ===
        self.vx += ax * dt
        self.vy += ay * dt

        # === Integriere Position ===
        # Translation muss rotiert werden, damit Bewegungsrichtung korrekt ist
        dx_world = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        dy_world = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)

        self.x += dx_world * dt
        self.y += dy_world * dt

        # === Integriere Rotation ===
        self.theta += wz * dt

        # === Pose veröffentlichen ===
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        self.pub_pose.publish(pose)

        rospy.loginfo(f"Pose: x={self.x:.2f} m, y={self.y:.2f} m, θ={math.degrees(self.theta):.1f}°")


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    rospy.spin()
