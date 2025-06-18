#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped


class LineFollowerNode(DTROS):
	def __init__(self, node_name):
       	 super(LineFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

       	 self.vehicle_name = os.environ['VEHICLE_NAME']
      	  twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
      	  self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

      	  # Abonnement à la position de la ligne détectée
          rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.callback_position, queue_size=1)

       	 self.base_speed = 0.3  # vitesse linéaire constante
       	 self.k_p = 2.0         # gain proportionnel à ajuster selon les performances
       	 self.error = 0.0

    def callback_position(self, msg):
        self.error = msg.data

   def run(self):
        rate = rospy.Rate(10)  # fréquence 10 Hz
        while not rospy.is_shutdown():
            # Calcul de la correction angulaire
            correction = self.k_p * self.error

            msg = Twist2DStamped()
            msg.v = self.base_speed         # vitesse linéaire constante
            msg.omega = correction          # correction angulaire (omega)
	          print (msg.omega)
            self._publisher.publish(msg)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    node = LineFollowerNode(node_name='line_follower_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()	

