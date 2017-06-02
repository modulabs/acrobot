#! /bin/bash

source ../../devel/setup.sh

roslaunch rrbot_gazebo rrbot_world.launch&

roslaunch acrobot_control acrobot_control.launch
