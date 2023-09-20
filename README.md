# Autonomous Driving Simulation: Bearing Only Visual Homing using 2-D Unicycle Model
Author: Hairuo Sun

Date: 6/27/2023

PI: Roberto Tron

## Project Overview
* This project utilizes the techinique of [Bearing-Only Visual Homing with Applications to a 2D Unicycle Model](https://drive.google.com/file/d/1WIOy5xXWTZDirO3twJEIU080zSkGeYzU/view?usp=share_link). The implementation of this method allow a 3D vehicle to traverse through an AR gate (with fiducial on all gateposts for gate localization) in a Gazebo simulation.

## setup
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
