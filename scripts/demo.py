#!/usr/bin/env python3
import argparse
from moveit_ros_planning_interface._moveit_move_group_interface import (
    MoveGroupInterface,
)
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
import moveit_commander
from moveit_commander import conversions

import rospy

import threading


def to_robot_state(move_group, joints):
    moveit_robot_state = move_group.get_current_state()
    moveit_robot_state.joint_state.header.stamp = rospy.Time.now()
    active_joints = move_group.get_active_joints()
    temp_joint_values = list(moveit_robot_state.joint_state.position)
    for i in range(len(active_joints)):
        temp_joint_values[
            moveit_robot_state.joint_state.name.index(active_joints[i])
        ] = joints[i]
    moveit_robot_state.joint_state.position = temp_joint_values
    return moveit_robot_state


def get_trajectory_joint_goal(plan):
    return plan.joint_trajectory.points[-1].positions


class MoveItDemo:
    def __init__(self) -> None:
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
        self.arms = {"left": self.left_group, "right": self.right_group}
        self.planning_scene_interface = moveit_commander.PlanningSceneInterface(
            synchronous=True
        )

    def go_to_named_pose(self, arm_name, goal_pose_stamped, plan_only=False):
        arm_group = self.arms[arm_name]

        arm_group.set_max_velocity_scaling_factor(1.0)
        arm_group.set_max_acceleration_scaling_factor(1.0)

        arm_group.set_named_target(goal_pose_stamped)

        # For now the only way to execute simultaneous motions is by planning first
        success, plan, time, error_code = arm_group.plan()
        arm_group.clear_pose_targets()
        if success:
            # Then executing that plan
            if not plan_only:
                return arm_group.execute(plan)
            else:
                return plan
        return None

    def go_to_goal_pose(
        self, arm_name, goal_pose_stamped, plan_only=False, initial_joints=None
    ):
        arm_group = self.arms[arm_name]

        arm_group.set_max_velocity_scaling_factor(1.0)
        arm_group.set_max_acceleration_scaling_factor(1.0)

        if initial_joints:
            arm_group.set_start_state(to_robot_state(arm_group, initial_joints))
        else:
            arm_group.set_start_state_to_current_state()

        arm_group.set_pose_target(goal_pose_stamped)

        # For now the only way to execute simultaneous motions is by planning first
        success, plan, time, error_code = arm_group.plan()
        arm_group.clear_pose_targets()
        if success:
            # Then executing that plan
            if not plan_only:
                return arm_group.execute(plan)
            else:
                return plan
        return None

    def go_to_joint_target(
        self, arm_name, joint_target, plan_only=False, initial_joints=None
    ):
        arm_group = self.arms[arm_name]

        arm_group.set_max_velocity_scaling_factor(1.0)
        arm_group.set_max_acceleration_scaling_factor(1.0)

        if initial_joints:
            arm_group.set_start_state(to_robot_state(arm_group, initial_joints))
        else:
            arm_group.set_start_state_to_current_state()

        arm_group.set_joint_value_target(joint_target)

        # For now the only way to execute simultaneous motions is by planning first
        success, plan, time, error_code = arm_group.plan()
        arm_group.clear_pose_targets()
        if success:
            # Then executing that plan
            if not plan_only:
                return arm_group.execute(plan)
            else:
                return plan
        return None

    def test_independent_workspace(self):
        """Plan and execute a simple series of motions for both arms in collision-free space"""
        self.go_to_named_pose("left", "home")
        self.go_to_named_pose("right", "home")

        def left_arm_motions():
            l_target1 = conversions.list_to_pose_stamped(
                [0.555, -0.000, 0.624, 1, 0, 0, 0], "panda_1_link0"
            )
            l_target2 = conversions.list_to_pose_stamped(
                [0.555, -0.250, 0.400, 1, 0, 0, 0], "panda_1_link0"
            )
            l_target3 = conversions.list_to_pose_stamped(
                [0.555, -0.000, 0.400, 1, 0, 0, 0], "panda_1_link0"
            )
            sequence = [l_target1, l_target2, l_target3]
            for _ in range(3):
                for target_pose in sequence:
                    self.go_to_goal_pose("left", target_pose)

        def right_arm_motions():
            r_target1 = conversions.list_to_pose_stamped(
                [0.555, -0.000, 0.624, 1, 0, 0, 0], "panda_2_link0"
            )
            r_target2 = conversions.list_to_pose_stamped(
                [0.555, -0.250, 0.400, 1, 0, 0, 0], "panda_2_link0"
            )
            r_target3 = conversions.list_to_pose_stamped(
                [0.555, -0.000, 0.400, 1, 0, 0, 0], "panda_2_link0"
            )
            sequence = [r_target1, r_target2, r_target3]
            for _ in range(3):
                for target_pose in sequence:
                    self.go_to_goal_pose("right", target_pose)

        left_thread = threading.Thread(target=left_arm_motions)
        right_thread = threading.Thread(target=right_arm_motions)
        left_thread.start()
        right_thread.start()
        left_thread.join(30)
        right_thread.join(30)

    def test_shared_workspace_online_planning(self):
        """ Plan and execute a simple series of motions for both arms.
            The motions will enter the other arms space.
            The plan and execution are done online, so the planning may give up due to collisions.
        """
        self.go_to_named_pose("left", "home")
        self.go_to_named_pose("right", "home")

        def left_arm_motions():
            l_target1 = conversions.list_to_pose_stamped(
                [0.056, 0.302, 0.547, 1, 0, 0, 0], "panda_1_link0"
            )
            l_target2 = conversions.list_to_pose_stamped(
                [0.245, 0.471, 0.200, 1, 0, 0, 0], "panda_1_link0"
            )
            l_target3 = conversions.list_to_pose_stamped(
                [0.245, 0.206, 0.200, 1, 0, 0, 0], "panda_1_link0"
            )
            sequence = [l_target1, l_target2, l_target3]
            for _ in range(3):
                for target_pose in sequence:
                    self.go_to_goal_pose("left", target_pose)

        def right_arm_motions():
            r_target1 = conversions.list_to_pose_stamped(
                [0.056, -0.302, 0.547, 1, 0, 0, 0], "panda_2_link0"
            )
            r_target2 = conversions.list_to_pose_stamped(
                [0.245, -0.471, 0.200, 1, 0, 0, 0], "panda_2_link0"
            )
            r_target3 = conversions.list_to_pose_stamped(
                [0.245, -0.206, 0.200, 1, 0, 0, 0], "panda_2_link0"
            )
            sequence = [r_target1, r_target2, r_target3]
            for _ in range(3):
                for target_pose in sequence:
                    self.go_to_goal_pose("right", target_pose)

        left_thread = threading.Thread(target=left_arm_motions)
        right_thread = threading.Thread(target=right_arm_motions)
        left_thread.start()
        right_thread.start()
        left_thread.join(30)
        right_thread.join(30)

    def test_shared_workspace_offline_planning(self, wait=True):
        """ Plan and execute a simple series of motions for both arms.
            The motions will enter the other arms space.
            The plan and execution are done offline, 
            so the planning always assumes a collision-free space but
            during execution the other arm might be in the way of the trajectory.
        """
        self.go_to_named_pose("left", "home")
        self.go_to_named_pose("right", "home")

        l_target1 = conversions.list_to_pose_stamped(
            [0.056, 0.302, 0.547, 1, 0, 0, 0], "panda_1_link0"
        )
        l_target2 = conversions.list_to_pose_stamped(
            [0.245, 0.471, 0.200, 1, 0, 0, 0], "panda_1_link0"
        )
        l_target3 = conversions.list_to_pose_stamped(
            [0.245, 0.206, 0.200, 1, 0, 0, 0], "panda_1_link0"
        )
        self.go_to_goal_pose("left", l_target1)
        plan0 = self.go_to_goal_pose("left", l_target2, plan_only=True)
        plan2 = self.go_to_goal_pose(
            "left",
            l_target3,
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan0),
        )
        plan3 = self.go_to_goal_pose(
            "left",
            l_target1,
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan2),
        )
        plan1 = self.go_to_joint_target(
            "left",
            get_trajectory_joint_goal(plan0),
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan3),
        )

        def left_arm_motions():
            self.left_group.execute(plan0, wait=wait)
            sequence = [plan2, plan3, plan1]
            for _ in range(3):
                for plan in sequence:
                    self.left_group.execute(plan, wait=wait)

        r_target1 = conversions.list_to_pose_stamped(
            [0.056, -0.302, 0.547, 1, 0, 0, 0], "panda_2_link0"
        )
        r_target2 = conversions.list_to_pose_stamped(
            [0.245, -0.471, 0.200, 1, 0, 0, 0], "panda_2_link0"
        )
        r_target3 = conversions.list_to_pose_stamped(
            [0.245, -0.206, 0.200, 1, 0, 0, 0], "panda_2_link0"
        )
        self.go_to_goal_pose("right", r_target1)
        plan0 = self.go_to_goal_pose("right", r_target2, plan_only=True)
        plan2 = self.go_to_goal_pose(
            "right",
            r_target3,
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan0),
        )
        plan3 = self.go_to_goal_pose(
            "right",
            r_target1,
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan2),
        )
        plan1 = self.go_to_joint_target(
            "right",
            get_trajectory_joint_goal(plan0),
            plan_only=True,
            initial_joints=get_trajectory_joint_goal(plan3),
        )

        def right_arm_motions():
            self.right_group.execute(plan0, wait=wait)
            sequence = [plan2, plan3, plan1]
            for _ in range(3):
                for plan in sequence:
                    self.right_group.execute(plan, wait=wait)

        left_thread = threading.Thread(target=left_arm_motions)
        right_thread = threading.Thread(target=right_arm_motions)
        left_thread.start()
        right_thread.start()
        left_thread.join(30)
        right_thread.join(30)


def main():
    """Main function to be run."""
    parser = argparse.ArgumentParser(description="Test MoveIt simultaneous motions")
    parser.add_argument(
        "-i",
        "--independent",
        action="store_true",
        help="Move both arms simultaneously in independent workspaces",
    )
    parser.add_argument(
        "-s",
        "--shared",
        action="store_true",
        help="Move both arms simultaneously in shared workspaces with online planning",
    )
    parser.add_argument(
        "-o",
        "--offline",
        action="store_true",
        help="Move both arms simultaneously in shared workspaces with offline planning",
    )
    args = parser.parse_args()

    rospy.init_node("moveit_simultaneous_motions_demo")

    demo = MoveItDemo()

    if args.independent:
        demo.test_independent_workspace()
    if args.shared:
        demo.test_shared_workspace_online_planning()
    if args.offline:
        demo.test_shared_workspace_offline_planning()



if __name__ == "__main__":
    main()
