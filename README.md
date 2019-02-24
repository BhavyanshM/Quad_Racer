# Titan AlphaPilot Racer

## Quad_Racer

This package named ```quad_racer``` is a C++ ROS node which generates ```rateThrust``` messages to control quadcopters. 

### Rostopics Published

```/uav/input/rateThrust```

### Requirements

This is a ROS package and expects ROS to be installed on the host machine running the ```quad_racer``` node. 

### Setup

Follow the steps provided below to download, build and launch the source code for ```quad_racer```

```
cd ~/catkin_ws/src
git clone https://github.com/alphapilotaichallenge/quad_racer.git
cd ..
catkin build
source devel/setup.bash
roslaunch quad_racer quad_racer_flightgoggles.launch ignore_collisions:=1 use_external_renderer:=1
```

### Author
Bhavyansh Mishra



