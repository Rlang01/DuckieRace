#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun detect_duckiebot_test_node.py

# wait for app to end
dt-launchfile-join