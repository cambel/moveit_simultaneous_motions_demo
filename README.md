# dual_arm_demo

![dual arm](https://github.com/bi3ri/dual_arm_demo/blob/main/dual_arm_demo.gif?raw=true)

Small demo project planning and executing random poses for two UR robots simultaneously. 

Tested successfully under noetic.

## Local Installation 
```bash
mkdir -p ~/dual_arm_ws/src
cd ~/dual_arm_ws/src
git clone https://github.com/bi3ri/dual_arm_demo.git
git submodule update --init --recursive
catkin build && source ~/dual_arm_ws/devel/setup.bash
```

## Docker Installation
```bash
git clone https://github.com/cambel/dual_arm_demo.git
cd dual_arm_demo/docker
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
roslaunch dual_arm_demo app.launch 
rosrun dual_arm_demo moveit_demo
```
