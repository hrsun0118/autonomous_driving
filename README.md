# Autonomous_driving Project Overview
* This project utilizes the techinique of [Bearing-Only Visual Homing with Applications to a 2D Unicycle Model]([https://drive.google.com/drive/u/3/folders/1goEMyYUSNCuOtoChnZRVUSiA3KM0IiBO](https://drive.google.com/file/d/1WIOy5xXWTZDirO3twJEIU080zSkGeYzU/view?usp=sharing)). The implementation of this method allow a 3D vehicle to traverse through an AR gate (with fiducial on all gateposts for gate localization) in a Gazebo simulation.

# setup
* Download the source code from this repo.
* Build 2 packages:
  * Create your own "ros_ws" folder, and cd to this folder
  * catkin build

* Launch 2 packages:
  * roslaunch me416_environment me416_environment.launch
  * roslaunch apriltag_ros continuous_detection_test.launch 
