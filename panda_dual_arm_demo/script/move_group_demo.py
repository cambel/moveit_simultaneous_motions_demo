#!/usr/bin/env python3

from copy import copy, deepcopy
import threading
import numpy as np
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, sqrt, tau, dist, fabs, cos


def get_trajectory_joint_goal(plan):
    return plan.joint_trajectory.points[-1].positions


def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class Arm:
    def __init__(self, group_name, end_effector):
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.end_effector = end_effector
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_max_acceleration_scaling_factor(1.0)

    def go_to_pose(self, pose_stamped, wait=True):
        group = self.group
        group.clear_pose_targets()

        group.set_end_effector_link(self.end_effector)

        # TODO add custom initial pose
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_stamped)
        success, plan, planning_time, error = group.plan()
        if not success:
            return False
        return self.execute_plan(plan, wait=wait)

    def execute_plan(self, plan, wait=True):
        if not self.group.execute(plan, wait=wait):
            return False
        self.group.clear_pose_targets()
        if wait:
            current_joints = self.group.get_current_joint_values()
            goal_joints = get_trajectory_joint_goal(
                plan, self.group.get_active_joints()
            )
            return all_close(goal_joints, current_joints, 0.01)
        return True


def main():
    rospy.init_node("simultaneous_trajectory_execution")
    panda1 = Arm("panda_1", "panda_1_hand")
    panda2 = Arm("panda_2", "panda_2_hand")

    # make one robot move continuously in a certain way, like circles
    def move_panda1():
        p1_pose1 = panda1.group.get_current_pose()

        p1_pose2 = deepcopy(p1_pose1)
        p1_pose2.pose.position.x -= 0.1

        p1_pose3 = deepcopy(p1_pose2)
        p1_pose3.pose.position.z -= 0.1

        p1_pose4 = deepcopy(p1_pose3)
        p1_pose4.pose.position.x += 0.1

        if not panda1.go_to_pose(p1_pose2):
            return
        if not panda1.go_to_pose(p1_pose3):
            return
        if not panda1.go_to_pose(p1_pose4):
            return
        if not panda1.go_to_pose(p1_pose1):
            return

    def move_panda2():
        pose1 = panda2.group.get_current_pose()

        pose2 = deepcopy(pose1)
        pose2.pose.position.x += 0.1

        pose3 = deepcopy(pose2)
        pose3.pose.position.z -= 0.1

        pose4 = deepcopy(pose3)
        pose4.pose.position.x -= 0.1

        panda2.go_to_pose(pose2)
        panda2.go_to_pose(pose3)
        panda2.go_to_pose(pose4)
        panda2.go_to_pose(pose1)

    t = threading.Thread(target=move_panda1)
    t2 = threading.Thread(target=move_panda2)
    t.start()
    # t2.start()
    t.join()
    # t2.join()
    print("done")
    # make the other robot do some random stuff in the meantime

    # show that rviz can also be used in the mean time.


if __name__ == "__main__":
    main()
