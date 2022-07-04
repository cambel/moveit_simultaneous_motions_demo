#include <ros/ros.h>

#include <memory>
// MoveitCpp
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <stdlib.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_moveit_demo_simple");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "dual_arm";

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");

  geometry_msgs::PoseStamped panda_1_pose_goal;
  geometry_msgs::PoseStamped panda_2_pose_goal;
  geometry_msgs::Pose panda_1_pose;
  geometry_msgs::Pose panda_2_pose;
  panda_1_pose_goal.header.frame_id = "base";
  panda_2_pose_goal.header.frame_id = "base";

  panda_1_pose.position.x = 0.732;
  panda_1_pose.position.y = -0.307;
  panda_1_pose.position.z = 0.395;
  panda_1_pose.orientation.x = 0.993;
  panda_1_pose.orientation.y = 0.000;
  panda_1_pose.orientation.z = 0.114;
  panda_1_pose.orientation.w = 0.000;

  panda_2_pose.position.x = 0.571;
  panda_2_pose.position.y = -0.002;
  panda_2_pose.position.z = 1.351;
  panda_2_pose.orientation.x = 0.993;
  panda_2_pose.orientation.y = 0.000;
  panda_2_pose.orientation.z = 0.114;
  panda_2_pose.orientation.w = 0.000;

  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  std::string panda_1_eff = "panda_1_hand_tcp";
  group.setPoseTarget(panda_1_pose, panda_1_eff);

  std::string panda_2_eff = "panda_2_hand_tcp";
  group.setPoseTarget(panda_2_pose, panda_2_eff);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto res = group.plan(my_plan);
  bool success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    ROS_INFO("Plan did not successed");
  } else {
    group.execute(my_plan);
  }

  ros::shutdown();
}