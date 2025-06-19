#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32

from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType

class WheelEncoderNode(DTROS):
    def __init__(self,node_name):
        super(WheelEncoderNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.sub_encoder_left = rospy.Subscriber(f"/{self._vehicle_name}/left_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.left_callback)
        self.sub_encoder_right = rospy.Subscriber(f"/{self._vehicle_name}/right_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.right_callback)

    def left_callback(self, msg):
        print(f"#ticks (left): {msg.data}")

    def right_callback(self, msg):
        print(f"#ticks(right): {msg.data}")


if __name__ == '__main__':
    # create the node
    node = WheelEncoderNode(node_name='wheel_encoder_node')
    # keep the process from terminating
    rospy.spin()




