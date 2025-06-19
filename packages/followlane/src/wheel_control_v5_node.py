#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64
import time

class TwistControlNode(DTROS):

    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

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

        # Souscription à l'erreur de vision
        rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.callback_position)

    def callback_position(self, msg):
        self._theta_hat = msg.data

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Temps écoulé depuis dernière exécution
            current_time = time.time()
            delta_t = current_time - self.last_time
            self.last_time = current_time

            # Calcul de l'erreur
            error = self._theta_ref - self._theta_hat
            self.integral_error += error * delta_t
            derivative_error = (error - self.prev_error) / delta_t if delta_t > 0 else 0.0
            self.prev_error = error

            # Calcul de la commande PID
            omega = self.k_p * error + self.k_i * self.integral_error + self.k_d * derivative_error

            # Création et publication du message
            msg = Twist2DStamped()
            msg.v = self._v
            msg.omega = omega
            self._publisher.publish(msg)

            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    node = TwistControlNode(node_name='twist_control_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
