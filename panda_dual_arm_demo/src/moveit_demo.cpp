#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <random>

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface dual_arm_group("dual_arm");

  dual_arm_group.setMaxVelocityScalingFactor(1.0);
  dual_arm_group.setMaxAccelerationScalingFactor(1.0);
  dual_arm_group.setPlanningTime(15.0);
  dual_arm_group.setNumPlanningAttempts(20.0);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  std::string panda_1_eff = "panda_1_link8";
  std::string panda_2_eff = "panda_2_link8";

  geometry_msgs::Pose panda_1_pose;
  geometry_msgs::Pose panda_2_pose;
  panda_1_pose.orientation.x = 0.993;
  panda_1_pose.orientation.y = 0.000;
  panda_1_pose.orientation.z = 0.114;
  panda_1_pose.orientation.w = 0.000;
  // w x y z
  Eigen::Quaternionf panda_1_q = Eigen::Quaternionf(0.000, 0.993, 0.000, 0.114);
  Eigen::Quaternionf panda_2_q = Eigen::Quaternionf(0.000, 0.993, 0.000, 0.114);

  for (int i; i < 100; i++) {
    float random_x = (((float)distr(gen)) * 0.01);
    float random_y = (((float)distr(gen)) * 0.01);
    float random_z = (((float)distr(gen)) * 0.01);

    panda_1_pose.position.x = 0.732 + random_x;
    panda_1_pose.position.y = 0.02029402510664674 + random_y;
    panda_1_pose.position.z = 1.658157440477098 + random_z;
    panda_2_pose.position.x = -0.01565011581780207 + random_x;
    panda_2_pose.position.y = -0.019683543216663102 + random_y;
    panda_2_pose.position.z = 1.657396455658871 + random_z;

    float x_rotation = rad_distr(gen) * 0.01;
    float y_rotation = rad_distr(gen) * 0.01;
    float z_rotation = rad_distr(gen) * 0.01;

    panda_1_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                panda_1_q;

    panda_2_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                panda_2_q;

    panda_1_pose.orientation.w = panda_1_q.w();
    panda_1_pose.orientation.x = panda_1_q.x();
    panda_1_pose.orientation.y = panda_1_q.y();
    panda_1_pose.orientation.z = panda_1_q.z();
    panda_2_pose.orientation.w = panda_2_q.w();
    panda_2_pose.orientation.x = panda_2_q.x();
    panda_2_pose.orientation.y = panda_2_q.y();
    panda_2_pose.orientation.z = panda_2_q.z();

    dual_arm_group.clearPoseTargets();
    dual_arm_group.setStartStateToCurrentState();
    std::string panda_1_eff = "panda_1_link8";
    dual_arm_group.setPoseTarget(panda_1_pose, panda_1_eff);

    std::string panda_2_eff = "panda_2_link8";
    dual_arm_group.setPoseTarget(panda_2_pose, panda_2_eff);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (dual_arm_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ROS_INFO("Plan did not succeeded");
    }
    dual_arm_group.execute(my_plan);
  }

  ros::shutdown();
}