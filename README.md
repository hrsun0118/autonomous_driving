# Autonomous_driving Project Overview
* This project utilizes the techinique of [Bearing-Only Visual Homing with Applications to a 2D Unicycle Model](https://drive.google.com/drive/u/3/folders/1goEMyYUSNCuOtoChnZRVUSiA3KM0IiBO). The implementation of this method allow a 3D vehicle to traverse through an AR gate (with fiducial on all gateposts for gate localization) in a Gazebo simulation.

# setup
* Build 2 packages:
  * Go to your own "ros_ws" folder
  * catkin build

* Launch 2 packages:
  * roslaunch me416_environment me416_environment.launch
  * roslaunch apriltag_ros continuous_detection_test.launch 
