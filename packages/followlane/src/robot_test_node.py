#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped

rospy.init_node('test_movement')
pub = rospy.Publisher('/<VEHICLE_NAME>/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
rospy.sleep(1)

msg = Twist2DStamped()
msg.v = 0.3
msg.omega = 0.0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
