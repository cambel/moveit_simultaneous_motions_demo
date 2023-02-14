#include <ros/ros.h>

#include <memory>
// MoveitCpp
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <stdlib.h>

namespace rvt = rviz_visual_tools;

moveit::core::RobotState createGoal(moveit::core::RobotState &start_state, std::vector<std::string> &joint_names,
                                    std::vector<double> &joint_positions) {
  moveit::core::RobotState rs = moveit::core::RobotState(start_state);
  trajectory_msgs::JointTrajectory jt;
  jt.joint_names = joint_names;
  trajectory_msgs::JointTrajectoryPoint jtp;
  jtp.positions = joint_positions;
  jt.points.push_back(jtp);
  jointTrajPointToRobotState(jt, 0, rs);
  return rs;
}

geometry_msgs::Pose robotStateToPose(moveit::core::RobotState &state, std::string &end_effector_link) {
  geometry_msgs::Pose pose;
  Eigen::Affine3d p = state.getGlobalLinkTransform(end_effector_link);
  tf::poseEigenToMsg(p, pose);
  return pose;
}

geometry_msgs::PoseStamped createTargetPose(std::string &frame_id, std::vector<double> &vector_pose) {
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = frame_id;
  target_pose.pose.position.x = vector_pose[0];
  target_pose.pose.position.y = vector_pose[1];
  target_pose.pose.position.z = vector_pose[2];
  target_pose.pose.orientation.x = vector_pose[3];
  target_pose.pose.orientation.y = vector_pose[4];
  target_pose.pose.orientation.z = vector_pose[5];
  target_pose.pose.orientation.w = vector_pose[6];
  return target_pose;
}

void visualize(moveit_visual_tools::MoveItVisualTools &visual_tools, moveit::core::RobotState &start_state,
               moveit::core::RobotState &target_state, const std::string &end_effector_link,
               robot_trajectory::RobotTrajectoryPtr &trajectory,
               const moveit::core::JointModelGroup *joint_model_group) {
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // Visualize the start pose in Rviz
  visual_tools.publishAxisLabeled(start_state.getGlobalLinkTransform(end_effector_link), "start_pose");
  visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
  // Visualize the goal pose in Rviz
  visual_tools.publishAxisLabeled(target_state.getGlobalLinkTransform(end_effector_link), "target_pose");
  visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
  // Visualize the trajectory in Rviz
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
}

int main(int argc, char **argv) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
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

  auto panda_1_planning_components = std::make_shared<moveit_cpp::PlanningComponent>("panda_1", moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto panda_1_start_state = panda_1_planning_components->getStartState();
  auto panda_1_joint_model_group_ptr = robot_model_ptr->getJointModelGroup("panda_1");
  std::vector<std::string> panda_1_joint_names = panda_1_joint_model_group_ptr->getJointModelNames();
  panda_1_joint_names.erase(panda_1_joint_names.end() - 1);

  std::vector<double> p1_target_joints1{1.057, -0.323, 0.805, -2.857, 0.424, 2.557, 1.468};
  std::vector<double> p1_target_joints2{0, 0, 0, -1.5707, 0, 1.8, 0};
  // panda 2

  auto panda_2_planning_components = std::make_shared<moveit_cpp::PlanningComponent>("panda_2", moveit_cpp_ptr);
  auto panda_2_start_state = panda_2_planning_components->getStartState();
  auto panda_2_joint_model_group_ptr = robot_model_ptr->getJointModelGroup("panda_2");
  std::vector<std::string> panda_2_joint_names = panda_2_joint_model_group_ptr->getJointModelNames();
  panda_2_joint_names.erase(panda_2_joint_names.end() - 1);

  std::vector<double> p2_target_joints1{2.049, 0.046, 2.419, -2.660, -0.053, 2.624, -1.666};
  std::vector<double> p2_target_joints2{0, 0, 0, -1.5707, 0, 1.8, 0};

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing
  // objects, robots, and trajectories in RViz as well as debugging tools such
  // as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools("base", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 2.25;
  visual_tools.publishText(text_pose, "MoveItCpp Simultaneous Trajectory Execution", rvt::WHITE, rvt::XLARGE);
  text_pose.translation().y() = 0.5;
  visual_tools.publishText(text_pose, "Panda 1", rvt::WHITE, rvt::XLARGE);
  text_pose.translation().y() = -0.5;
  visual_tools.publishText(text_pose, "Panda 2", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning with MoveItCpp
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // There are multiple ways to set the start and the goal states of the plan
  // they are illustrated in the following plan examples
  //
  // Plan #1
  // ^^^^^^^

  // Panda 1
  //
  // We can set the start state of the plan to the current state of the robot
  panda_1_planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using
  auto p1_target = createGoal(*panda_1_start_state, panda_1_joint_names, p1_target_joints1);
  panda_1_planning_components->setGoal(p1_target);

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto p1_plan = panda_1_planning_components->plan();

  // Panda 2
  //
  auto p2_target = createGoal(*panda_2_start_state, panda_2_joint_names, p2_target_joints1);
  panda_2_planning_components->setGoal(p2_target);

  auto p2_plan = panda_2_planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (p1_plan && p2_plan) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Visualizing plan 1.");
    // Panda 1
    visualize(visual_tools, *panda_1_start_state, p1_target, "panda_1_link8", p1_plan.trajectory_,
              panda_1_joint_model_group_ptr);
    visualize(visual_tools, *panda_2_start_state, p2_target, "panda_2_link8", p2_plan.trajectory_,
              panda_2_joint_model_group_ptr);

    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    ROS_INFO_STREAM_NAMED(LOGNAME, "Executing plan 1. (Non-blocking)");
    panda_1_planning_components->execute(false); // Execute the plan
    panda_2_planning_components->execute(false); // Execute the plan
  }

  // Start the next plan
  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /////////////

  // Panda 1
  panda_1_planning_components->setStartStateToCurrentState();
  panda_1_start_state = panda_1_planning_components->getStartState();
  p1_target = createGoal(*panda_1_start_state, panda_1_joint_names, p1_target_joints2);
  panda_1_planning_components->setGoal(p1_target);
  p1_plan = panda_1_planning_components->plan();

  // Panda 2
  panda_2_planning_components->setStartStateToCurrentState();
  panda_2_start_state = panda_2_planning_components->getStartState();
  p2_target = createGoal(*panda_2_start_state, panda_2_joint_names, p2_target_joints2);
  panda_2_planning_components->setGoal(p2_target);
  p2_plan = panda_2_planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (p1_plan && p2_plan) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Visualizing plan 2.");
    // Panda 1
    visualize(visual_tools, *panda_1_start_state, p1_target, "panda_1_link8", p1_plan.trajectory_,
              panda_1_joint_model_group_ptr);
    visualize(visual_tools, *panda_2_start_state, p2_target, "panda_2_link8", p2_plan.trajectory_,
              panda_2_joint_model_group_ptr);

    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    ROS_INFO_STREAM_NAMED(LOGNAME, "Executing plan 2. (Blocking)");
    panda_1_planning_components->execute(true); // Execute the plan
    panda_2_planning_components->execute(true); // Execute the plan
  }

  // Start the next plan
  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  auto robot_model = moveit_cpp_ptr->getRobotModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  // Make sure the Panda robot is in "ready" state which is loaded from
  // fake_controller.yaml
  std::vector<double> joints_vals;
  robot_state->copyJointGroupPositions("panda_1", joints_vals);

  robot_state->printStatePositions();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Panda 1 joint configuration: ");

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
}