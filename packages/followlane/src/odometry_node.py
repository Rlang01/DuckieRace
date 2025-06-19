#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Pose2D
import os
from duckietown.dtros import DTROS, NodeType


class OdometryNode(DTROS):
    def __init__(self, node_name):
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control)
        self.sub_imu = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/data", Imu, self.data_callback)
        self.sub_temperature = rospy.Subscriber(f"/{self._vehicle_name}/imu_node/temperature", Temperature, self.temperature_callback)

        # Pose publisher
        self.pub_pose = rospy.Publisher(f"/{self._vehicle_name}/odometry/pose", Pose2D, queue_size=1)

        # Initial pose and velocity
        self.x = 0.0
        self.y = 0.0
        self.alpha = 0.0  # orientation
        self.vx = 0.0
        self.vy = 0.0

        # Time
        self.last_time = None

    def cb_control(self, msg):
        print(f"Control mode: {msg.data}")

    def data_callback(self, msg: Imu):
        current_time = msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Read linear acceleration (in m/s^2)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        # Optional: Compensate for gravity if necessary

        # Read angular velocity (in rad/s)
        angular_vel_z = msg.angular_velocity.z

        # Integrate to get velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # Integrate velocity to get position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Integrate angular velocity to get orientation
        self.alpha += angular_vel_z * dt

        # Publish pose
        print(f"Pose: x={round(self.x,2)}, y={round(self.y,2)}, alpha={round(self.alpha,2)}")
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.alpha
        self.pub_pose.publish(pose)

    def temperature_callback(self, msg: Temperature):
        print(f"Temperature: {msg.temperature}")


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    rospy.spin()
