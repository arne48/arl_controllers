# Controllers of the Adaptive Robotics Laboratory

## Index
1. Introduction
2. Controllers
	1. MuscleController
3. Usage
	1. Config Files
	2. Launch Files
4. Build Status

## Introduction
The history of ROS origins in the laboratories of [Stanford](http://stair.stanford.edu/) and was later developed under the supervision of [Willow Garage](http://www.willowgarage.com/) as a middleware for Robotic Systems.

Naturally the controller primitives which where implemented as the base of this system reflects the needs of Willow Garage and their developers community.
Thinking of the [PR2](http://wiki.ros.org/Robots/PR2)  and the [Turtlebot](http://wiki.ros.org/Robots/TurtleBot) as the most prominent robots supported in every major version of ROS the default controllers implements the utilization of revolute and prismatic joints by the robot described by effort, velocity and position. Additionally sensor, joint state and locomotion controllers are available by default.

In the ARL [Pneumatic Artificial Muscles](http://lucy.vub.ac.be/publications/Daerden_Lefeber_EJMEE.pdf) are used to driver the robots. With this kind of actuation the default controllers of ROS are not straightforward to use and and by wrapping them these might also reduce the advantages of this way of driving a robot.

Hence new base types of controllers starting with an essential one called the **MuscleController** were implemented.

## Controllers
The following paragraphs will describe the new controllers.

### MuscleController
#### Intro
The **MuscleController** implements the usage of PAMs based on sensed information like the *tension* and *current pressure* within a muscle and being able to set a *desired pressure*.

#### Utilized Interface and Handle 
*MuscleHandle* from [arl_interfaces](https://github.com/arne48/arl_interfaces)

*MuscleInterface* from [arl_interfaces](https://github.com/arne48/arl_interfaces)

#### Further information about controlling strategies
TBA

## Usage
### Config-Files
* muscle_controller.yaml
Contains the details of the **MuscleController** for the Parameter Server so it will be loaded by the controller manager once it is started by the *arl_driver_node* from [arl_hw](https://github.com/arne48/arl_hw).

### Launch-Files
* muscle_controller.launch
Loads the details of the **MuscleController** found in the *muscle_controller.yaml* and calls the spawner of the controller manager afterwards.

## Build Status
Once tests are added to the project a status indicator will be placed here.