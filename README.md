dvrk robot
==========

This package contains programs, which fires real robot, publishes and
subscribes ROS 2 topics.

# Depends

* sawIntuititiveResearchKit 
* cisst-ros2

IMPORTANT: You first need to make sure you have all your configuration
files ready, very likely in
~/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share.  You don't
need to copy your configuration files back and forth anymore.

# How to Run

For the Qt based application without rviz:
```sh
  ros2 run dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>
```

We also provide a launch script for single arm using RViz (you need to provide your own console_<arm>.json file):
```sh
  ros2 launch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/<user_name>/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-PSM1.json
```

One can also simulate one or more arms using the field `simulation` in your console-xyz.json.  See examples in the `sawIntuitiveResearchKit/share` directory:
```sh
  ros2 launch dvrk_robot dvrk_arm_rviz.launch arm:=ECM
```

We provide a few console configurations for simulated arms in `sawIntuitiveResearchKit/share/console`.  By default, the launch file `dvrk_arm_rviz.launch` will look for the file `console/console-$(arg arm)_KIN_SIMULATED.json`.

# Using the ROS 2 topics

The best way to figure how to use the ROS topics is to look at the
python ROS wrappers in `dvrk_python` or the Matlab ROS wrappers in
`dvrk_matlab`.  The ROS topics are described in https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/API-2.x
