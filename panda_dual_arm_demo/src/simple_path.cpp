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

  // Create MoveGroups
  moveit::planning_interface::MoveGroupInterface dual_arm_group("dual_arm");
  moveit::planning_interface::MoveGroupInterface panda_1_group("panda_1");
  moveit::planning_interface::MoveGroupInterface panda_2_group("panda_2");
  
  dual_arm_group.setPlanningTime(0.5);

  ROS_INFO_STREAM("Panda_1 initial pose" << panda_1_group.getCurrentPose("panda_1_link8") << "\n");
  ROS_INFO_STREAM("Panda_2 initial pose" << panda_2_group.getCurrentPose("panda_2_link8") << "\n");

  geometry_msgs::PoseStamped panda_1_pose_goal;
  geometry_msgs::PoseStamped panda_2_pose_goal;
  geometry_msgs::Pose panda_1_pose;
  geometry_msgs::Pose panda_2_pose;
  panda_1_pose_goal.header.frame_id = "base";
  panda_2_pose_goal.header.frame_id = "base";

  panda_1_pose.position.x = 0.450;
  panda_1_pose.position.y =-0.499924;
  panda_1_pose.position.z = 1.600;
  panda_1_pose.orientation.x = 0.993436;
  panda_1_pose.orientation.y = 3.5161e-05;
  panda_1_pose.orientation.z = 0.114386;
  panda_1_pose.orientation.w = 2.77577e-05;

  panda_2_pose.position.x = 0.450;
  panda_2_pose.position.y = 0.499964;
  panda_2_pose.position.z = 1.600;
  panda_2_pose.orientation.x = 0.993434;
  panda_2_pose.orientation.y = -7.54803e-06;
  panda_2_pose.orientation.z = 0.114403;
  panda_2_pose.orientation.w = 3.67256e-05;

  dual_arm_group.clearPoseTargets();
  dual_arm_group.setStartStateToCurrentState();
  std::string panda_1_eff = "panda_1_link8";
  dual_arm_group.setPoseTarget(panda_1_pose, panda_1_eff);

  std::string panda_2_eff = "panda_2_link8";
  dual_arm_group.setPoseTarget(panda_2_pose, panda_2_eff);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto res = dual_arm_group.plan(my_plan);
  bool success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    ROS_INFO("Plan did not succeeded");
  } else {
    dual_arm_group.execute(my_plan);
  }

  ros::shutdown();
}