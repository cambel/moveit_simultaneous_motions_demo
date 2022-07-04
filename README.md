# dual_arm_demo

Small demo project planning and executing random poses for two panda robots simultaneously.
As part of the GSoC 2022 MoveIt's Simultaneous Trajectory Execution [project](https://github.com/ros-planning/moveit/issues/3156).

Developed on ROS Noetic.

## Docker Installation
```bash
git clone https://github.com/cambel/moveit_simultaneous_motions_demo.git
cd moveit_simultaneous_motions_demo/docker
bash launch_container.sh
```
Inside the docker:

```bash
apt update
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build && source devel/setup.bash
```

## Run demo 
#run in two shells
```bash
roslaunch moveit_resources_dual_panda_moveit_config demo.launch 
rosrun panda_dual_arm_demo simple_path
```
