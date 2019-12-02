# pacman
[![Build Status](https://travis-ci.org/cyhap/pacman.svg?branch=master)](https://travis-ci.org/cyhap/pacman)
[![Coverage Status](https://coveralls.io/repos/github/cyhap/pacman/badge.svg?branch=master)](https://coveralls.io/github/cyhap/pacman?branch=master)

Codename: Pac-man 

## Project Overview
This final project for 'ENPM808X: Software Development for Robotics' encompasses an autonomous robot product idea and follows high-quality software engineering practices and agile development utilized to build a simulated robotic system and demo using ROS and Gazebo. For our software design project, we are developing a collection robot. This robot will act vaguely similar to the arcade game character, Pac-man (which is also what we have used for the codename of our project), moving around a defined enclosed map, though in this case, autonomously, and collecting specific objects while ignoring others. The operating scenario for our design is one of the assembly rooms at the fictional ACME Robotics headquarters. The robot will maneuver its way through the design space, which will contain permanent objects like tables and walls with which it will have to avoid colliding. The source of the problem for which we have created a solution is that ACME has many employees that drop items on the floor throughout the day. Some of these items are potentially hazardous to the workflow (think banana peels), while others are important tools that have been placed there on purpose (think tools). Our collection robot will search the space, utilizing localization techniques and image processing to find and collect the waste objects, while ignoring other objects and avoiding all obstacles.

## Personnel
* Corbyn Yhap</br> Corbyn is aquiring his Masters in Engineering for Robotics at the University of Maryland, College Park. Both prior to returning to school to obtain his Masters, and concurrently, Corbyn works as an Software Engineer in Industry.
* Ethan Quist</br> Ethan is aquiring his Masters in Engineering for Robotics at the University of Maryland, College Park. Prior to returning to school to obtain his Masters, Ethan worked as an Engineer in Industry.
* Ari Kupferberg</br> Ari is aquiring his Masters in Engineering for Robotics at the University of Maryland, College Park, where he also works as a full-time Teaching Assistant for a joint graduate/undergraduate Medical Robotics course. 

## Licensing
This software operates with a 3-clause BSD 2.0 license and can be found inside the file LICENSE.

## Assumptions/Dependencies
The code assumes you are using *Ros Kinetic*.

The package builds using *catkin*.

This package relies on the following dependencies that must be installed:
* roscpp
* geometry_msgs
* std_msgs
* sensor_msgs
* kobuki_msgs
* cv_bridge
* image_transport

The following programs/packages are used and must be installed:
* Gazebo
* Turtlebot_Gazebo

## Download Package
```
cd <your catkin workspace>/src
git clone https://github.com/cyhap/pacman.git
```

## Build
```
cd <your catkin workspace>
catkin_make
source devel/setup.bash
```

## Test
```
cd <your catkin workspace>
catkin_make run_tests
```


## Run 
*TO DO*

## Demo
_TO DO_

## Product Backlog / Worklog
The link below is for the AIP Workbook for our project:</br>
https://drive.google.com/file/d/1Yul22CLmPJAjaOQzCEN-TskSz7LMM9vA/view?usp=sharing

## Sprint Planning
Sprint Planning Document for Iteration 1:</br>
https://docs.google.com/document/d/1PFSHEmctJcphnvS_sIFLBwKtJKjMI6X8Y2TYh6Sgb3E/edit?usp=sharing


Sprint Planning Document for Iteration 2:</br>
https://docs.google.com/document/d/1PW3oNJy1oSttl2ZZlZhgE-w6Ei8aKcm3vlHW0Bm25Wg/edit?usp=sharing
