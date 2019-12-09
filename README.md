# pacman
[![Build Status](https://travis-ci.org/cyhap/pacman.svg?branch=master)](https://travis-ci.org/cyhap/pacman)
[![Coverage Status](https://coveralls.io/repos/github/cyhap/pacman/badge.svg?branch=master)](https://coveralls.io/github/cyhap/pacman?branch=master)

Codename: Pac-man

## Project Overview
This final project for 'ENPM808X: Software Development for Robotics' encompasses an autonomous robot product idea and follows high-quality software engineering practices and agile development utilized to build a simulated robotic system and demo using ROS and Gazebo. For our software design project, we are developing a collection robot. This robot will act vaguely similar to the arcade game character, Pac-man (which is also what we have used for the codename of our project), moving around a defined enclosed map, though in this case, autonomously, and collecting specific objects while ignoring others. The operating scenario for our design is one of the assembly rooms at the fictional ACME Robotics headquarters. The robot will maneuver its way through the design space, which will contain permanent objects like outer walls and hallways with which it will have to avoid colliding. The source of the problem for which we have created a solution is that ACME has many employees that drop items on the floor throughout the day. Some of these items are potentially hazardous to the workflow (think banana peels), while others are important tools that have been placed there on purpose (think tools). Our collection robot will search the space, utilizing localization techniques and image processing to find and collect the waste objects, while ignoring other objects and avoiding all obstacles. To maintain the Pac-man connection, our designed objects to be collected are strawberries, which gives our robot power, and ghosts, which the robot actively avoids.</br>
We have created a demonstration of our robot's performance in the Gazebo Simulator, with a link down below. This project was completed over the course of three weeks, with a one-week sprint schedule turnover, and the sprint planning/review documents are also provided below, along with the AIP spreadsheet with product and integration backlogs and complete worklog.

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

Our demonstration runs on Gazebo 7.16.0, issues may occur if using an older version of gazebo
In order to update gazebo follow the code below:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install gazebo7 -y
```


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
We have created unit tests for both Level 1 and Level 2 Integration Tests. These tests can be run together, and the Level 2 Integration Test can also be run on its own.

To run the unit tests, the ROS master must be initiated, which can be done using 'roscore'.</br>
Open two terminals:

Terminal 1:
```
roscore
```
Terminal 2:
```
cd <your catkin workspace>
catkin_make run_tests
```

To check only the Level 2 Integration Test for ROS, run the following in a terminal:
```
rostest pacman testsrv.test
```

## Run 
After the respository has been downloaded with all listed dependancies above, it can be run through the following code:
```
cd <your workspace>
roslaunch pacman demo.launch
```
This code will run and launch all necessary nodes to display the software working and the simulation in Gazebo.

A neat function that can be used to see the published mask images the software produces can be activated by running the following code in a separate terminal while the launch above is running:
```
rqt_image_view
```


## Demo
The demonstration of our software is run through the same launch file stated above. It will open up Gazebo and automatically start the nodes required to make the turtlebot collect the objects. Make sure the dependencies on the correct Gazebo version are followed.
```
cd <your workspace>
roslaunch pacman demo.launch
```

## Rosbag and Logging
We have provided the option to use rosbag and record the messages passed between nodes in our software. To run our software with rosbag  logging enabled run the following code:
```
cd <your workspace>
roslaunch pacman demo.launch recordChatter:=1
```

## Simulation Demo Video
A video of our software demo can be found [here](https://drive.google.com/file/d/1BmlBf0pP5Lmpbd15uag93hTwoebfn4Qc/view?usp=sharing).

## Product Backlog / Worklog
The link for the AIP Workbook for our project is [here](https://drive.google.com/file/d/1Yul22CLmPJAjaOQzCEN-TskSz7LMM9vA/view?usp=sharing).

## Sprint Planning and Review
Sprint 1 Planning and Review can be found [here](https://docs.google.com/document/d/1PFSHEmctJcphnvS_sIFLBwKtJKjMI6X8Y2TYh6Sgb3E/edit?usp=sharing).

Sprint 2 Planning and Review can be found [here](https://docs.google.com/document/d/1PW3oNJy1oSttl2ZZlZhgE-w6Ei8aKcm3vlHW0Bm25Wg/edit?usp=sharing).

Sprint 3 Planning and Review can be found [here](https://docs.google.com/document/d/1YeXOjUnyZIUHrRWdbr8HYcAxKHh3Rp9Pjmb5Gm5eai4/edit?usp=sharing).

## Final Presentation
A link to our final presentation can be found [here](https://docs.google.com/presentation/d/1-ndrX1u3dt5uddjnWisbmw2-8t7fMI8LAkdIyzekoBo/edit?usp=sharing).
