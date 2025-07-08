#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64
import time

class wheel_control_node(DTROS):

    def __init__(self, node_name):
        super(wheel_control_node, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Paramètres de régulation de vitesse en fonction de la distance
        self.safe_distance = 0.5       # au-delà de cette distance : pleine vitesse
        self.stop_distance = 0.15      # en-dessous : arrêt complet
        self.max_speed_factor = 1.0
        self.min_speed_factor = 0.0
        self.speed_factor = 1.0        # sera multiplié à self._v

        # Abonnement au capteur de proximité avant
        tof_topic = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        rospy.Subscriber(tof_topic, Float64, self.callback_proximity)

        # Initialisation des variables PID
        self._v = 0.3  # vitesse linéaire constante
        self.k_p = 3.0
        self.k_i = 0.5
        self.k_d = 0.2

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

        self._theta_ref = 0.0  # direction idéale (ex: 0 radian = tout droit)
        self._theta_hat = 0.0  # angle mesuré reçu de la vision
        self._theta_hat_smoothed = 0.0  # angle lissé

        # Coefficient de lissage EWMA
        self.ewma_alpha = 0.9

        # Abonnement à la détection de ligne
        rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.callback_position)

    def callback_proximity(self, msg):
        distance = msg.data

        if distance <= self.stop_distance:
            self.speed_factor = self.min_speed_factor
        elif distance <= self.safe_distance:
            # Interpolation linéaire entre stop_distance et safe_distance
            scale = (distance - self.stop_distance) / (self.safe_distance - self.stop_distance)
            self.speed_factor = scale * (self.max_speed_factor - self.min_speed_factor)
        else:
            self.speed_factor = self.max_speed_factor

        rospy.loginfo(f"[TOF] distance = {round(distance, 2)} m → speed factor = {round(self.speed_factor, 2)}")

    def ewma_filter(self, new_value):
        self._theta_hat_smoothed = self.ewma_alpha * self._theta_hat_smoothed + (1 - self.ewma_alpha) * new_value
        return self._theta_hat_smoothed

    def callback_position(self, msg):
        self._theta_hat = msg.data
        self.ewma_filter(self._theta_hat)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_time = time.time()
            delta_t = current_time - self.last_time
            self.last_time = current_time

            error = self._theta_ref - self._theta_hat_smoothed
            self.integral_error += error * delta_t
            derivative_error = (error - self.prev_error) / delta_t if delta_t > 0 else 0.0
            self.prev_error = error

            omega = self.k_p * error + self.k_i * self.integral_error + self.k_d * derivative_error

            msg = Twist2DStamped()
            msg.v = self._v * self.speed_factor  # vitesse ajustée
            msg.omega = omega
            self._publisher.publish(msg)

            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    node = wheel_control_node(node_name='wheel_control_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
