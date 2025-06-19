#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from enum import Enum

import os
from duckietown.dtros import DTROS, NodeType

class ControlType(Enum):
    followLane = 1
    wait = 2
    bypassObstacle = 3



class SwitchControlNode(DTROS):
    def __init__(self,node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Float64, self.cbDuckieDetected, queue_size = 1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size = 1)
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        
        self._control_mode = ControlType.followLane



    def cbDuckieDetected(self, msg):
        print('Duckiebot detected')
        # if duckiebot detected and in followLane-mode --> switch to bypassObstacle
        if self._control_mode == ControlType.followLane:
            self._control_mode = ControlType.bypassObstacle

    def cbLaneDetected(self, msg):
        print('Lane detected')
        # Write your own code her

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            msg_control = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()