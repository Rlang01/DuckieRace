#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun packages steuerung_pid scripts pid_control_V3.py

dt-launchfile-join
