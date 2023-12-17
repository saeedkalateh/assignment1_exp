# assignment1 experimental robotics lab

## Introduction
This package is an experiment to use a topogical map ontology for controling a robot in ROS through aRMOR service. The ontology consists of an indoor 
environment with multiple rooms and a mobile robot. 

<p align="center">
<img src="https://github.com/saeedkalateh/assignment1_exp/assets/83904267/572b9b2d-51da-4fec-b228-1acc90fb24cc" width="300" title="topological map">
</p>

The robot start in the E location and waits until it receives the information to build the
topological map, i.e., the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.
The robot moves in a new location, and waits for some times before to visit another location. This
behavior is repeated in a infinite loop. 

When the robot battery is low, it goes in the E
location, and wait for some times before to start again with the above behavior.

When robot battery is not low, it should move among locations with this policy:

1. It should mainly stay on corridors,
2. If a reachable room has not been visited for some times it should visit it.

## Software Architecture
In order to simulate the movements of the robot and its stimulus, the approach presented in the 
[arch_skeleton](https://github.com/buoncubi/arch_skeleton) example is used with some changes
(e.g. planner and controller delay time, battery level).

Other main part of software architucture consists of finite state machine which is based on [smach](http://wiki.ros.org/smach) for having a better 
overview of process states and transitions among them and [aRMOR](https://github.com/EmaroLab/armor)
for using the ontology of topogical map for controling the robot in ROS.

The software architucture is represented in the following figure.

<p align="center">
<img src="https://github.com/saeedkalateh/assignment1_exp/assets/83904267/cd5e9135-4c84-44cd-86cf-7804a5e19f0d" width="620" title="com1">
</p>

The components of this software architucture can be described as follows:

### robot-state
The ``robot-state`` is a node that encodes the knowledge shared among the other components,
and it implements two services for robot position (i.e., ``state/set_pose`` and ``state/get_pose``) 
and two other for robot battery level (i.e., ``state/set_battery_level`` and ``state/get_battery_level``)

### motion planner
The planner node implements an action server named ``motion/planner``. This is done by the means of the 
``SimpleActionServer`` class based on the ``Plan`` action message. This action server requires the ``state/get_pose/`` 
service of the ``robot-state`` node, and a ``target`` point given as goal.

Given the current and target points, this component returns a plan as a list of ``via_points``, which only 
consist of the starting and ending points for simplicity. Morever the delay time for generating the path
is considered to be small to prevent conflicts between robot real situation and finite state machine.
When a new ``via_points`` is generated, the updated plan is provided as ``feedback``. When all the 
``via_points`` have been generated, the plan is provided as ``results``.

### motion controller
The ``controller`` node implements an action server named ``motion/controller``. This is done by the means of the 
``SimpleActionServer`` class based on the ``Control`` action message. This action server requires the 
``state/set_pose/`` service of the ``robot-state`` node and a plan given as a list of ``via_points`` by the 
``planner``. 

Given the plan and the current robot position, this component iterates for each planned ``via_point`` and 
waits to simulate the time spent moving the robot to that location. The waiting time is computed using the 
robot speed and the eucledian distance between the points. Each time a ``via_point`` is reached the 
``state/set_pose`` service is invoked, and a ``feedback`` is provided. When the last ``via_point`` is reached, 
the action service provides a result by propagating the current robot position, which has been already updated 
through the ``state/set_pose`` service. Morever in each movement step, the robot battery level is read by the
``controller`` node using ``state/get_battery_level`` and after decreasing gets set through ``state/set_battery_level``.


### finite state machin - aRMOR
Defines the states and transitions for the finite state machine of the topological map, it also 
uses ``helper.py`` helper script to update the ontology while the process is running, and 
retreives the target room based on last visit times, finaly sends the target room pose to the 
``motion_planner`` through ``/target_point`` in order to find the path.


### Temporal Diagram (UML Sequence Diagram)
The following figure represents the UML sequence diagram of this package. In the initial state, user launches the nodes, causing some of them
to get some rosparams such as ``environment_size`` and ``initial_pose``. Then, in each loop cycle, ``armor_service`` which is implemented inside
``finite_state_machine`` node, gets the current robot pose from ``robot-state`` node and updates the ontology, then the target room will be sent to the ``planner_client`` through ``/target_point`` topic and also the state of gets changed considering the target room. ``Planner`` tries to find the path and 
sends it to the ``planner_client`` as ``PlanResult``. Finally the ``controller_client`` node subscribes the ``/path`` topic and sends it as ``ControlGoal`` to the ``controller`` node so that it can move the robot by setting the robot pose through ``/state/set_pose`` service and the cycles goes on again.

<p align="center">
<img src="https://github.com/saeedkalateh/assignment1_exp/assets/83904267/3ee61dcc-b3e3-445c-92d4-d0fda9e260b4" width="820" title="seq1">
</p>



## Usage
### Installation
This package is based on [aRMOR](https://github.com/EmaroLab/armor) it has to be installed as it is described
in the provided link as a pre-condition for running this package.

It is also depended on [smach](http://wiki.ros.org/smach), it can be installed using the following commands:

```bashscript
$ sudo apt-get install ros-<distro>-executive-smach*
```
```bashscript
$ sudo apt-get install ros-<distro>-smach-viewer
```

Once the dependencies are met, the package can be installed as it follows:

```bashscript
$ mkdir -p catkin_ws/src
```
```bashscript
$ cd catkin_ws/src
```
```bashscript
$ git clone https://github.com/saeedkalateh/assignment1_exp
```
```bashscript
$ cd ..
```
```bashscript
$ source /opt/ros/<distro>/setup.bash
```
```bashscript
$ catkin_make
```

### Running
In order to initialize the software architucture along with the finite state machine representation, run the following command.

```bashscript
$ source devel/setup.bash
```
```bashscript
$ roslaunch assignment1 fsm.launch
```

## Working Hypothesis and Environment

1. System's Features: The topological map is considered to be in size 10x10. Each time when a room is chosen as a target room from the ontology, the corresponding point will be sent to the planner as ``PlanGoal``

2. System's Limitations: The planner is the main limitation in this rospackage, because it only considers the starting and target point. The battery level is also a limitation in the robot behaviour, causing it to select the charging room "E" over the target room resulted from the last visit times.

3. Possible Technical Improvements: The planner node could be improved in such a way that it considers the walls more precisely. The battery level could also implemented in a more realistic way.

## Authors and Contacts
- Saeed Kalateh
- email: 4982001@studenti.unige.it
