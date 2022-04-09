#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>


#include <moveit/move_group_interface/move_group_interface.h>


#include <stdlib.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo_simple");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "rocket_and_groot";

  // ros::NodeHandle nodeHandle("/move_group");
  // auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nodeHandle);
  // moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
  // auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  // auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  // auto robot_start_state = planning_components->getStartState();
  // auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group("rocket_and_groot");
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  // group.setEndEffectorLink("groot_tool0");
  // group.setEndEffectorLink("rocket_tool0");

  geometry_msgs::PoseStamped rocket_pose_goal;
  geometry_msgs::PoseStamped groot_pose_goal;
  geometry_msgs::Pose rocket_pose;
  geometry_msgs::Pose groot_pose;
  rocket_pose_goal.header.frame_id = "world";
  groot_pose_goal.header.frame_id = "world";
 
  rocket_pose.position.x = 0.0;
  rocket_pose.position.y = 0.8;
  rocket_pose.position.z = 1.4;
  rocket_pose.orientation.x = 0.7082493155730533;
  rocket_pose.orientation.y = 0.7058456228553038;
  rocket_pose.orientation.z = 0.006125141443269484;
  rocket_pose.orientation.w = 0.011284783055502262;
 
  groot_pose.position.x = 0.0;
  groot_pose.position.y = -0.8;
  groot_pose.position.z = 1.4;
  groot_pose.orientation.x = 0.7060029188906415;
  groot_pose.orientation.y = -0.7080875314562214;
  groot_pose.orientation.z = -0.011505315872495573;
  groot_pose.orientation.w = 0.006289198740942946;

  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  std::string rocket_eff = "rocket_tool0";
  group.setPoseTarget(rocket_pose, rocket_eff);

  std::string groot_eff = "groot_tool0";
  group.setPoseTarget(groot_pose, groot_eff);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto res = group.plan(my_plan);
  bool success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_INFO("Plan did not successed");
  } else {
    group.execute(my_plan);
  }

  ros::shutdown();
}