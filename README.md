# Project: Autonomous Driving Simulation
## Bearing Only Visual Homing using 2-D Unicycle Model
Author: Hairuo Sun

Date: 6/27/2023

PI: Roberto Tron

## Summary
* This  utilizes the techinique of [Bearing-Only Visual Homing with Applications to a 2D Unicycle Model](https://drive.google.com/file/d/1WIOy5xXWTZDirO3twJEIU080zSkGeYzU/view?usp=share_link). The implementation of this method allow a 3D vehicle to traverse through an AR gate (with fiducial on all gateposts for gate localization) in a Gazebo simulation.

## Summary
[Project General Description - 2 - 3 sentences]

### Project Deliverables/Functionalities
* deliverable 1
* deliverable 2
* deliverable 3

## Solution Design
<div align="center">
<img src="./images/solution_design.png">
<p> Solution Design (HW/SW) </p>
<br/>
<br/>
<br/>
</div>

## Setup
* Download the source code from this repo.
* Run the code in a Linux based OS.
* Build 2 packages:
  * Create your own "ros_ws" folder, and cd to this folder
  * catkin build

## Execution
* Launch the me416_environment world:
  * Run in terminal: roslaunch me416_environment me416_environment.launch
* Launch the camera recording:
  * Run in terminal: rosrun rqt_image rqt_image
  * Place the rover at a location where camera can see both images
* Launch the continuous_detection script:
  * roslaunch apriltag_ros continuous_detection_test.launch
* Observe the rover behaviors:
  * if the "2 apriltag (on 2 gateposts)" system is selected --> rover will drive towards the gate & stop right before the gate.
  * if the "2 apriltag (on 3 gateposts)" system is selected --> rover will drive through the gate & stop right before the furtherest gate in the middle.

## DEMO Videos
* [Visual Homing in Gazebo Simulation - using 3 Tag](https://youtu.be/CaGOPrl82sU)

## Technical Components
### Linux System
* [VMWare Workstation(virtual machine)](https://www.vmware.com/products/workstation-player.html)
* [Ubuntu OS](https://ubuntu.com/desktop)

### SW Components/Libraries
* [ROS](https://www.ros.org)
* [Gazebo](https://gazebosim.org/home)
* [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)
* [me416_environment](./Working%20ME416%20Ccode%20for%20Autonomous%20Navigation(work%20with%203%20apriltags)%206_27_2023/me416_environment/)
* [me415_lab](./Working%20ME416%20Ccode%20for%20Autonomous%20Navigation(work%20with%203%20apriltags)%206_27_2023/me416_lab/)
* [ROS library - geometry_msgs](http://wiki.ros.org/geometry_msgs)
* [squaternion](https://pypi.org/project/squaternion/)
* [numpy](https://numpy.org/install/)

## Other Sketches and photos (detailed design diagrams, setup images, terminal screen output, GUI page, etc)
<div align="center">
<img src="./images/pic1.png">
<p> Pic 1</p>
<br/>
<br/>
<br/>
</div>
<div align="center">
<img src="./images/pic2.png">
<p> Pic 2</p>
<br/>
<br/>
<br/>
</div>

## References (includes any papers, datasheets, forum posts, tutorials, debug site links, articles used during the project completion process)
### Datasheets
* [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
* [module 2](link)

### tutorials/debug/articles
* [tutorial 1](link)
* [Stackoverflow debug question](link)
* [Nordic forum post 1](link)


## Future Improvements:
* Improvement 1
* Improvement 2
