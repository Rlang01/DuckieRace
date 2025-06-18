f#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped

# throttle and direction for each wheel
THROTTLE_LEFT = 0.4        # 40% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.4       # 40% throttle
DIRECTION_RIGHT = 1        # forward

class LineFollowerNode(DTROS):
    def __init__(self, node_name):
        super(LineFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Abonnement à la position de la ligne détectée
        rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.callback_position, queue_size=1)

        self.base_speed = 0.4  # vitesse de base constante
        self.k_p = 0.6         # coefficient proportionnel à ajuster
        self.error = 0.0

    def callback_position(self, msg):
        self.error = msg.data

    def run(self):
        rate = rospy.Rate(10)  # fréquence 10 Hz
        while not rospy.is_shutdown():
            # Calcul de la correction proportionnelle
            correction = self.k_p * self.error

            # Ajustement des vitesses des roues
            v_left = THROTTLE_LEFT * DIRECTION_LEFT - correction
            v_right = THROTTLE_RIGHT * DIRECTION_RIGHT + correction

            # Limiter les vitesses entre -1 et 1 (optionnel mais recommandé)
            v_left = max(min(v_left, 1.0), -1.0)
            v_right = max(min(v_right, 1.0), -1.0)

            msg = WheelsCmdStamped()
            msg.vel_left = v_left
            msg.vel_right = v_right

            self._publisher.publish(msg)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    node = LineFollowerNode(node_name='line_follower_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
