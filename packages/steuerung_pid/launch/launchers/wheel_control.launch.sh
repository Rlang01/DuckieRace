#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun steuerung_pid pid_control_V3.py

dt-launchfile-join
