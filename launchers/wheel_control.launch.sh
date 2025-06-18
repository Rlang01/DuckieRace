#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun steuerung_pid pid_control_V4_node.py

dt-launchfile-join
