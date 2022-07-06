#include <ros/ros.h>

#include <memory>
// MoveitCpp
#include <geometry_msgs/PointStamped.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <stdlib.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_path_moveitcpp");

  ros::NodeHandle nh("/simple_path_moveitcpp");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  static const std::string LOGNAME = "simple_path_moveitcpp";

  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting MoveIt CPP...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  // panda 1

  auto panda_1_planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
      "panda_1", moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto panda_1_robot_start_state = panda_1_planning_components->getStartState();
  auto panda_1_joint_model_group_ptr =
      robot_model_ptr->getJointModelGroup("panda_1");
  
  // panda 2

  // auto panda_2_planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
  //     "panda_2", moveit_cpp_ptr);
  // auto panda_2_robot_start_state = panda_2_planning_components->getStartState();
  // auto panda_2_joint_model_group_ptr =
  //     robot_model_ptr->getJointModelGroup("panda_2");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing
  // objects, robots, and trajectories in RViz as well as debugging tools such
  // as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools(
      "base", rvt::RVIZ_MARKER_TOPIC,
      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning with MoveItCpp
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // There are multiple ways to set the start and the goal states of the plan
  // they are illustrated in the following plan examples
  //
  // Plan #1
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  panda_1_planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using
  // geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "base";
  target_pose1.pose.position.x = 0.450;
  target_pose1.pose.position.y = -0.50;
  target_pose1.pose.position.z = 1.50;
  target_pose1.pose.orientation.x = 0.993436;
  target_pose1.pose.orientation.y = 3.5161e-05;
  target_pose1.pose.orientation.z = 0.114386;
  target_pose1.pose.orientation.w = 2.77577e-05;
  panda_1_planning_components->setGoal(target_pose1, "panda_1_link8");

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = panda_1_planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (plan_solution1) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Visualizing plan 1.");
    // Visualize the start pose in Rviz
    visual_tools.publishAxisLabeled(
        panda_1_robot_start_state->getGlobalLinkTransform("panda_1_link8"), "start_pose");
    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the goal pose in Rviz
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the trajectory in Rviz
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory,
                                       panda_1_joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    ROS_INFO_STREAM_NAMED(LOGNAME, "Executing plan 1.");
    panda_1_planning_components->execute(); // Execute the plan
    //   panda_1_planning_components->execute("panda_1", plan_solution1.trajectory, false);
  }

  // Start the next plan
  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
}