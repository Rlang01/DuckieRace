#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64

VELOCITY = 0.3  # vitesse linéaire
KP = 4.0         # gain proportionnel

class TwistControlNode(DTROS):
    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        vision_topic = f"/{vehicle_name}/detect/lane"

        self._v = VELOCITY
        self.k_p = KP
        self.error = 0.0
        self._omega = 0.0

        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        rospy.Subscriber(vision_topic, Float64, self.callback_position)

    def callback_position(self, msg):
        self.error = msg.data
        self._omega = self.k_p * self.error

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Twist2DStamped()
            msg.v = self._v
            msg.omega = self._omega
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
