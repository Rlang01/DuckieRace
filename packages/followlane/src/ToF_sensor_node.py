#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32

import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Range

class ToFSensorNode(DTROS):
    def __init__(self,node_name):
        super(ToFSensorNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_ToF = rospy.Subscriber(f"/{self._vehicle_name}/front_center_tof_driver_node/range", self.cb_ToF, queue_size = 1)

    def cb_ToF(self, msg):
        distance: str = f"{msg.range:.3f}m" if msg.range < msg.max_range else "Too-far"
        print(f"Range: {distance}")

if __name__ == '__main__':
    # create the node
    node = ToFSensorNode(node_name='ToF_sensor_node')
    # keep the process from terminating
    rospy.spin()