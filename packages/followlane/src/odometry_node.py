#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Float64, Int32
import os
from duckietown.dtros import DTROS, NodeType


class OdometryNode(DTROS):
    def __init__(self,node_name):
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control)
        self.sub_imu = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/data", Imu, self.data_callback)
        self.sub_temperature = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/temperature", Temperature, self.temperature_callback)

    def cb_control(self, msg):
        print(f"Control mode: {msg.data}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            msg_control = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)
    
    def data_callback(msg = Imu):
        print(f"Linear Acceleration: {msg.linear_acceleration}, Angular Velocity: {msg.angular_velocity}")

    def temperature_callback(msg: Temperature):
        print(f"Temperature: {msg.temperature}")


if __name__ == '__main__':
    # create the node
    node = OdometryNode(node_name='odometry_node')
    node.run()
    # keep the process from terminating
    rospy.spin()
