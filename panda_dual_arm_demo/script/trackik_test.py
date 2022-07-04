#!/usr/bin/env python

import numpy as np
import rospy
import rostest
import os

# from moveit_ros_planning_interface._moveit_move_group_interface import (
#     MoveGroupInterface,
# )
import moveit_commander



def main():
    rospy.init_node("test")
    rocket_group = moveit_commander.MoveGroupCommander("rocket")
    
    rocket_pose = rocket_group.get_current_pose("rocket_tool0")
    # rocket_pose = conversions.list_to_pose_stamped(rocket_pose, rocket_group.get_planning_frame())
    print(rocket_pose)
    rocket_pose.pose.position.z -= 0.05
    rocket_pose.pose.position.x += 0.05
    
    rocket_pose.set_start_state_to_current_state()
    rocket_pose.set_pose_target(rocket_pose, end_effector_link="rocket_tool0")

    rocket_pose.go()        


if __name__ == "__main__":
    main()    
