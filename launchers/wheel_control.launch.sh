#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun steuerung_pid wheel_control_V5_node.py


dt-launchfile-join
