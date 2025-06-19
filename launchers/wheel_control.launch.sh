#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun followlane wheel_control_v5_node.py



dt-launchfile-join
