Experiment control algorithm using acrobot mechanism
Acrobot is double pendulum mechanism having 1 passive and 1 active joint

## Introduction
Set up torque control environment in gazebo using ros_control
Now, Just set torque = sin(i)
Later, going to apply control algorithm here

## Usage
- Download rrbot tutorial in gazebo
```sh
cd ~/catkin_ws/src/
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
cd ..
catkin_make
```

- Download acrobot_control in this repository at ~/catkin_ws/src/

- Authorize scripts/acrobot_control.py
```sh
chmod +x acrobot_contro.py
```

- Launch rrbot_gazebo
```sh
roslaunch rrbot_gazebo rrbot_world.launch
```

- Launch acrobot_control
```sh
roslaunch acrobot_control acrobot_control.launch
```

## To do
1. Explain above structure in detail
2. Make above two launch files into one launch file for simplicity
3. Add LQR algorithm

## Reference
1. <http://gazebosim.org/tutorials?cat=connect_ros>
2. <https://github.com/JoshMarino/gazebo_and_ros_control>
