# 16 Ways to Gallop
## Video


https://github.com/user-attachments/assets/f61d39dd-1e16-44e3-9111-f9862808ee01


## Overview

This repository contains the code used to formulate a nonlinear programming (NLP) problem designed to identify eight optimal asymmetrical gaits for the A1 quadrupedal robot. The targeted gaits are: Transverse and Rotary Galloping without Flight Phases (TG0 and RG0); Transverse and Rotary Galloping with Gathered Suspension (TGG and RGG); Transverse and Rotary Galloping with Extended Suspension (TGE and RGE); and Transverse and Rotary Galloping with Dual Flight Phases (TG2 and RG2). The code leverages the Fast Robot Optimization and Simulation (FROST) toolkit (https://github.com/ayonga/frost-dev). Additionally, the repository includes the code used to conduct simulation tests for the A1 robot in the Gazebo environment, which employs a Quadratic Programming (QP)-based locomotion controller.

*This is research code—subject to frequent updates—and is provided without any warranty regarding its fitness for any particular purpose.
**The source code is released under a BSD 3-Clause license.

Author: Yasser G. Alqaham, Jing Cheng, Zhenyu Gan
Affiliation: DLAR Lab
Maintainer: Yasser G. Alqaham, ygalqaha@syr.edu, and Jing Cheng jcheng13@syr.edu.

This projected was initially developed at Syracuse University (Dynamic Locomotion and Robotics Lab).

## Publications

This work has been submitted to the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2025).

## Optimization Requirements
### Environment

To use this code, you need MATLAB R2019b and Wolfram Mathematica 12.1.

## Usage

1- Download FROST framework from (https://github.com/ayonga/frost-dev) and follow the steps on the installation page.

2- Add the "Optimization" folder to the main folder of FROST.

3- For each gait, run the main file section by section. For example, RG0 main file name is "A1_RG0."

## Simulation Requirements
### Environment

We recommend that users run this project in Ubuntu 18.04 with ROS melodic or 20.04 with ROS noetic.

### Dependencies

Please place the three packages, unitree_guide, unitree_ros, and unitree_ros_to_real in our repository in a ROS workspace’s source directory.

## build

Open a terminal and switch the directory to the ros workspace containing unitree_guide, then run the following command to build the project:
```
catkin_make
```

## run

In the same terminal, run the following command step by step:
```
source ./devel/setup.bash
```
To open the gazebo simulator, run:
```
roslaunch unitree_guide gazeboSim.launch 
```

For starting the controller, open an another terminal and switch to the same directory,  then run the following command:
```
./devel/lib/unitree_guide/junior_ctrl
```

## Usage

After starting the controller,  the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from **Passive**(initial state) to **FixedStand**.  

Then press the ‘7’ key to switch the FSM from **FixedStand** to **Galloping_G0**, which corresponds to the ground Transverse Galloping Gait. The robot will accelerate itself.

Press the '2' to return to the **FixedStand**. The program automatically creates a plot including front/hip position, joint angle, foot position, ground reaction force, motor torque, etc. After you close all the figures, you will return to the **FixedStand**.

Go back to **FixedStand**, then press the ‘7’ key to switch the FSM from **FixedStand** to **Galloping_G2**,  which corresponds to the Transverse Galloping Gait with two fly phase. The robot will accelerate itself.(If there is no response, you need to click on the terminal opened to start the controller and then repeat the previous operation)

If you want to change the Transverse Galloping Gait to Rotory Galloping Gait, plase modify the vector **_bias** by switching the last two elemets in the State_Galloping_G0.cpp and State_Galloping_G2.cpp; 

