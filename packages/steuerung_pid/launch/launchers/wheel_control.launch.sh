#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun steuerung_pid wheel_control_v3.py

dt-launchfile-join
