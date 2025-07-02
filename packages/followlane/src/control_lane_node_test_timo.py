#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType

class ControlLaneNode(DTROS):
    def __init__(self, node_name):
        super(ControlLaneNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ["VEHICLE_NAME"]
        self.enable = False

        # 💡 Hardcodierte PID-Parameter
        self.v_bar = 0.25         # konstante Vorwärtsgeschwindigkeit (m/s)
        self.k_p = 2.5            # proportionaler Verstärkungsfaktor
        self.k_i = 0.1            # integraler Verstärkungsfaktor
        self.k_d = 0.05           # differenzieller Verstärkungsfaktor
        self.integral_limit = 1.0 # Begrenzung für die Integralsumme (Anti-Windup)

        # Publisher: Fahrkommandos
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Subscriber: Spurfehler & Moduswahl
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cb_lane)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control)

        # PID-Zustände
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("control_lane_node_test_timo gestartet mit festem PID-Regler.")

    def cb_control(self, msg):
        self.enable = (msg.data == ControlType.followlane.value)

    def cb_lane(self, msg):
        if not self.enable:
            return

        error = msg.data
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # PID-Berechnung
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        omega = - (self.k_p * error + self.k_i * self.integral + self.k_d * derivative)
        v = self.v_bar

        # Kommando senden
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        self.pub_cmd.publish(twist)

    def on_shutdown(self):
        rospy.loginfo("Node wird beendet – Stoppe Duckiebot.")
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd.publish(stop)

if __name__ == '__main__':
    node = ControlLaneNode(node_name='control_lane_node_test_timo')
    rospy.spin()
