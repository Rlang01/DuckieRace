#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun steuerung_pid wheel_control_V4_node.py


dt-launchfile-join
