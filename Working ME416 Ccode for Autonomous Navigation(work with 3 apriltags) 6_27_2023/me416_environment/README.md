# Gazebo environment for ME 416

## Run the environment

To launch our Gazebo environment, run:

```
roslaunch me416_environment me416_environment.launch
```

## ROSBot model

The ROSBot SRDF (i.e. it's model description) can be found in `/models/diff_drive/`. The two important files are:

1. `diff_drive.xacro` contains the different links and joints that build the robot.
2. `parameters.xacro` has the dimensions of each link.
