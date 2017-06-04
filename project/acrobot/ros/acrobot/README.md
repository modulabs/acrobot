Experiment control algorithm using acrobot mechanism
Acrobot is double pendulum mechanism having 1 passive and 1 active joint

## Introduction
Acrobot control algorithm implemented.
In swing-up phase, task space partial feedback linearization used,
Near origin, lqr used,
Switching between 2 phases, region of attraction algorithm have to be implemented (not yet)

## Usage
- Prerequisite

- Download and build 
```sh
cd ~/catkin_ws/
catkin_make
```

- Download acrobot_gazebo, acrobot_description, acrobot_control package in this repository at ~/catkin_ws/src/

- If you want to use python code, authorize acrobot_control/scripts/acrobot_control.py
```sh
chmod +x acrobot_control.py
```

- Launch rrbot_gazebo
```sh
roslaunch rrbot_gazebo rrbot_world.launch
```

- Launch acrobot_control
```sh
roslaunch acrobot_control acrobot_control.launch
```
or if you want to use python code
```sh
roslaunch acrobot_control acrobot_control_python.launch
```


## To do
1. Installation prerequisite
2. Region of attraction 

## Reference
1. tedrake lecture
2. tedrake paper
3. spong swing up
4. sos
5. <http://gazebosim.org/tutorials?cat=connect_ros>
6. <https://github.com/JoshMarino/gazebo_and_ros_control>
