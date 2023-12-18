# Experimental Robotics Laboratory assignment 
This is the second assignment of the university course Experimental robotics laboratory, developed by the student [Gabriele Russo](https://github.com/GabrieleRusso11).

The full code documentation is here https://gabrielerusso11.github.io/exproblab_assignment_2/

# Introduction
The aim of this assignment is to simulate a surveillance robot in an indoor environment (the pictures below). The robot starts in the E corridor and then it will go to visit the other locations, giving the priority to the urgent locations and to the corridors. The urgent locations are location that the robot hasn't visited for some time. Once it is in a location, it has to scan the location rotating a camera. During this process if the robot gets a low battery level, it has to go in the corridor E which is the charging location.

In order to do this, the indoor environment is associated to an ontology implemented using the [Armor system](https://github.com/EmaroLab/armor) and the robot behaviour is defined using a Final State Machine which is implemented through [ROS SMACH](http://wiki.ros.org/smach).

Unlike the [assignment 1](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele) now the environment is simulated using Gazebo, and in order to know the environment informations (like the name of the locations and the coordinates) the robot has to scan with a camera, the markers which sorround it in its initial position (x = -6.0, y = 11.0). The marker are detected using [aruco_ros](https://github.com/CarmineD8/aruco_ros)

The code of this assignment has been developed using as landmark the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) by [Professor Luca Buoncomapgni](https://github.com/buoncubi).

![Assignment environment](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/assignment_map.png)

# Software Architecture
This software architecture is composed of four python scripts:

- `state_machine.py`
- `robot_state.py`
- `controller.py`

 and one c++ script:

- `marker_server.cpp`

Then there are other two python scripts:

- `state_machine_interface.py` : which is an interface for the `state_machine.py` in order to communicate with the others nodes, and it also contains many methods used in the state machine node to compute data and to interact with the ontology

- `architecture_names.py` : contains all the names of the nodes and the names of topics, services and actions.

In order to explain in efficient way the software architecture, in the following, are exploited three diagrams :

- **Component diagram**: explain the structure of the architecture and the topics, services and actions that are used

- **State diagram** : explain the structure of the Finite State Machine node

- **Temporal diagram** : explain the temporal workflow of the architecture  

## Component diagram

![Component diagram](https://github.com/GabrieleRusso11/exproblab_assignment_2/blob/main/media/assignment2_component_diagram.png)

The architecture is composed of 9 components:

- **Armor server**: this component is the interface between the state machine node and the ontology associated to the indoor environment.
It is implemented by the [Armor system](https://github.com/EmaroLab/armor).
In order to load the ontology, do changes and queries, is used the [ArmorPy API](https://github.com/EmaroLab/armor_py_api).

- **State Machine**: this component is the heart of the architecture. It handles all the other nodes in order to define the robot behaviour in the most efficient way.
It is implemented using the [ROS SMACH](http://wiki.ros.org/smach), and has an interface (`state_machine_interface.py`) used to communicates with the other nodes (components) and also to contains methods used to compute data and to interact with the ontology. (see the state diagrams to understand the composition of this component).

- **Robot State**: this component contains the state of the robot, so its actual position and its battery level.

- **Controller**: this component is an action server that takes the target location by the state machine and send a [move base](http://wiki.ros.org/move_base) goal in order to move the robot towards the target location, avoiding walls and obstacles. Then it sends the new location reached to the State Machine and Robot State components.

- **Aruco_ros**: this component use the [aruco_ros](https://github.com/CarmineD8/aruco_ros) package in order to detect the marker and sends the marker id detected to the state machine component.

- **Marker_server**: this component is developed by the [Professor Carmine Recchiuto](https://github.com/CarmineD8). It receives by the state machine component, the marker id detected and returns the location informations associated to the specific marker id (location name, x and y coordinates).

- **Move_base**: this component is the [move base](http://wiki.ros.org/move_base) package, used to move the robot in the simulated environment. It receives the goal from the controller component and return the goal status (succeed if the robot has reached the target location). State machine components sends a cancel goal request when the robot is moving and it gets a low battery level.

- **Slam Gmapping**: this component is the [slam gmapping](http://wiki.ros.org/gmapping) package in order to build the map using the hokuyo laser scan (gazebo plugin)

- **Gazebo**: this component is the [gazebo](https://gazebosim.org/home) robot simulator, used to simulate the robot (for example camera, hokuyo laser, manipulator, joint position controller) and also the indoor environment.

## State diagram

![State diagram](https://github.com/GabrieleRusso11/exproblab_assignment_2/blob/main/media/state_diagram_2.png)

The State machine componet is described by this state diagram.
The state machine is composed by 5 states:

- **WaitForTopMap**: in this state the robot waits that the camera has scanned and processed al the markers, then build (with the informations acquired by the markers) and load the ontology through the armor server. So, in a nutshell waits that the ontology of the indoor environment is ready.

- **Planning**: once the ontology is ready, the robot passes to the planning state. In this state the robot plans in which location has to go.

- **Moving**: in this state the robot is moved towards the target location, using the controller node, exploiting the move_base package.

- **Scan**: when the robot has reached the target location it rotates the manipulator on itself, where is attached the camera, in order to scan the location reached.

- **Recharging**: in this state the robot has a low battery level, so starts to reach the charging location. When it is in the charging location it starts the charging process.

## Temporal diagram

The temporal diagram for this second assignment is quite similar to the [first assignment](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele) but in this case the ontology is not defined by the user but is defined detecting the markers id.
So in this case the first step is use aruco_ros to detect the markers id, that will be sent to the state machine. Then the state machine will send this markers id to the marker_server in order to takes the location informations.
Then the state machine can build the ontology as the first assignment (in this case there is an additional property, the x and y coordinates of the location).
Al the other things are the same, but know there is not anymore the planner node, because the planning process is done by slam gmapping and move_base packages.
Another new think is the comunication between the state machine and Gazebo. The state machine will send the joint position commands to gazebo in order to control the robot manipulator, where is attached the camera.

## list of ros parameters

- `starting_map`: is the file used as ontology base

- `operative_map`: is the final ontology file

- `charging station`: indicates the charging location
- `location_property`: the name of the property that adds doors to the locations in the ontology

- `robot_property`: the name of the property that set and get the robot position in the ontology

- `urgent_property`: the name of the property that sets the urgency threshold of a location

- `visited_property`: the name of the property that updates the location timestamp

- `canreach_property`: the name of the property that gets the locations which the robot can reach

- `now_property`: the name of the property that updates the robot timestamp

- `coord_x_property`: the name of the property that set and get the x coordinate of the location

- `coord_y_property`: the name of the property that set and get the y coordinate of the location

- `robot_name`: the name of the robot

- `battery_time`: the time used to define the charging and discharging time of the battery

- `charging_location`: indicates the name of the charging location

# Installation and running procedure.
This simulation run on Ros Noetic.
Firstly, to run this project is needed the [Armor system](https://github.com/EmaroLab/armor) and the [ROS SMACH](http://wiki.ros.org/smach)
In order to install the Armor, follow this [Tutorial](https://github.com/EmaroLab/armor/issues/7).

To install Ros Smach for the final state machine : 

`sudo apt-get install ros-noetic-executive-smach*`

Then clone this repositories:
- aruco_ros:

`git clone https://github.com/CarmineD8/aruco_ros`

- slam_gmapping:

`git clone https://github.com/CarmineD8/SLAM_packages.git`

- move_base:

`git clone https://github.com/CarmineD8/planning`

in order to properly load the markers in the robotic simulation later, copy the folder `models` of the aruco_ros package in the hidden folder `/root/.gazebo/models`

to control the robot in the gazebo simulation environment is needed th **ros_control** packages.
So open the terminal and execute the following lines:

`
sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers
`

`
sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control
`

Then in the workspace src folder clone this repository : 

`git clone https://github.com/GabrieleRusso11/exproblab_assignment_2.git`

Then go in the exproblab_assignment_2 folder and clone this repository for the ontology:

`git clone https://github.com/buoncubi/topological_map`

Substitute the `aruco_ros/src/marker_publish.cpp` with the `marker_publish.cpp` of this repository.

Lastly run:

`catkin_make`

In order to run this project, go in the workspace and launch the launch file:

`roslaunch exproblab_assignment_2 assignment.launch`

# Video Demo

https://github.com/GabrieleRusso11/exproblab_assignment_2/assets/93495918/58c60e15-1564-401e-9fff-0d341a6f9e92

The video has a velocity x1.5.

# Working hypothesis and environment 
For this project are considerated different assumptions.
Firstly, the robot is spawned in the gazebo simulation environment in a room with 7 markers.
The robot has to use a manipulator with a camera, in order to scan and detect all the marker id, also the markers that are on the walls. In this case only the camera has to rotate, not the robot
Then, the robot starts from the E corridor which is both the charging location and a normal location to be visited.
Then, there is a priority in the choice of the location to visit. Robot has to visit before the corridors and then the rooms.
If there are no corridors between the urgent locations to visit, then the robot will give higher priority to the urgent room, otherwise the corridors have always the higher priority.
In this way, since the corridors are connected to more locations (a corridors has to have at least 2 doors) than the rooms, the robot can reach more location and can reach quickly the charging location when the battery is low.
The battery level low is considered has a minimum battery level threshold after that the robot has to charge is battery as soon as possible. 
When the battery level is low, if the robot is already in the charging location, the charging process starts immediatly, otherwise the robot is in the low battery level mode and the move_base and the controller will be used only to find and reach the charging location.
So when the robot is in the rescharging state, it can go in the planning state both if the battery is charged and also if the robot has to reach the charging location (low battery mode)

 
## System’s features

In the following are listed the system's features:

- Colored User Interface in order to allow the users to modify some ontology parameters and to see all the outputs of the nodes.

- High priority to battery_low signals in order to recharge the robot's battery as soon as possible

- possibilty of setting the parameters saved in the ROS parameter server for a tailored functioning of the system and for passing the ontology folder path from the launch file

- python script to manage the ontology and interact with it and to function as interface between the state machine node and the other nodes

- robot_state node that keeps track of the robot state (position and battery level)

- Slam_gmapping and move_base for plan and move the robot towards the goal location avoiding the walls and the obstacles

- Log informations for each node with different colors

- 3D simulation environment using Gazebo

- 3D robot model (robot_model.xacro and robot_model.gazebo) with a camera and hokuyo laser scan gazebo plugins, four wheels, skid_steer_drive controller, and a manipulator with a revolute base joint and a prismatic joint on the link 1 in order to move up and down the camera on it.

## System’s limitations and Possible technical Improvements

- **Manipulator movements**: the movement of the robot manipulator are not smooth, especially when there is the rotation of the joint base. A possible improvement is to modify parameters in the robot_model.xacro and choosing better tuning parameters for the joint position controllers.

- **Robot motion**: the robot moves very slowly. A possible improvement is to modify some move_base parameters in order to make the robot faster and more agile.

- **Battery management**: Actually the battery level is only a boolean variable that passes from True to false (and viceversa) in order to simulate the change of the battery level. The charging and discharging of the battery are a simple busy waiting times.
So a first improvement is to output the charging and the discharging of the battery in order to show to the users the increasing or decreasing of the battery. Lastly, the battery management can be improved with a more realistic and proper algorithm, in order to make the battery usage more realistic.

- **Urgent Locations management**: the urgent location to visit is chosen randomly between the locations in the urgent location list (if there are corridors, the priority is given to them), this means that there is not an order based on the time (for example first urgent location to visit is the location that the robot has not visited in the longest time). So a possible improvement is to handle the urgent location list giving higher priority based on the less recent visited time.


# Authors and contacts

**Institution**: University of Genoa

**Course**: MSc in Robotics Engineering

**Subject**: Experimental Robotics Laboratory

**Author**: Gabriele Russo

**Student Id**: S5180813

**E-mail**: gabriele.russo117@gmail.com
