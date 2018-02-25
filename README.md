Experiment control algorithm using acrobot mechanism </br>
Acrobot is double pendulum mechanism having 1 passive and 1 active joint

## Introduction
Acrobot control algorithm implemented.</br>
In swing-up phase, task space partial feedback linearization used,</br>
Near origin, lqr used,</br>
Switching between 2 phases, region of attraction algorithm have to be implemented (not yet)

## Usage
### Prerequisite</br>
Install gazebo-ros-pkgs and gazebo-ros-control (might already installed)</br>

    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

Install effort-controllers for torque-control</br>

    $ sudo apt-get install ros-kinetic-effort-controllers

### Download and build 

    $ cd ~/catkin_ws/src
    $ git clone 
    $ cd ~/catkin_ws/
    $ catkin_make

### Run

    $ roslaunch acrobot_gazebo acrobot_world.launch

## To do
1. write a lqr controller using ros-control
2. kdl parser and generalize
3. automatically switching controller using region of attraction algorithm

## Reference
1. Tedrake Lecture
2. [Tedrake paper](http://groups.csail.mit.edu/robotics-center/public_papers/Majumdar13.pdf)
3. [Spong paper](http://ieeecss.org/CSM/library/1995/feb1995/02-swingupctrlprob.pdf)
4. sos
5. [gazebo tutorial](http://gazebosim.org/tutorials?cat=connect_ros)
6. [gazebo and ros control](https://github.com/JoshMarino/gazebo_and_ros_control)
