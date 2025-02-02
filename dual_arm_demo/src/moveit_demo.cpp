#include <ros/ros.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface dual_arm_group("rocket_and_groot");

  dual_arm_group.setMaxVelocityScalingFactor(1.0);
  dual_arm_group.setMaxAccelerationScalingFactor(1.0);
  dual_arm_group.setPlanningTime(15.0);
  dual_arm_group.setNumPlanningAttempts(20.0);
  
  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  std::string rocket_eff = "rocket_tool0";
  std::string groot_eff = "groot_tool0";
  

  geometry_msgs::Pose rocket_pose;
  geometry_msgs::Pose groot_pose;
  
  // w x y z
  Eigen::Quaternionf rocket_q = Eigen::Quaternionf(0.0044319521005895665 , -0.0018064082028716572, 0.714190127940822, -0.6999353940485185);
  Eigen::Quaternionf groot_q = Eigen::Quaternionf(0.7171097271676862 , -0.6959453209354478, -0.029260144371181365, -0.02361341612136324);

  for(int i; i < 100; i++){

    float random_x = ( ((float) distr(gen)) * 0.01);
    float random_y = ( ((float) distr(gen)) * 0.01);
    float random_z = ( ((float) distr(gen)) * 0.01);

    rocket_pose.position.x = -0.015463195119993365 + random_x;
    rocket_pose.position.y = 0.02029402510664674 + random_y;
    rocket_pose.position.z = 1.658157440477098 + random_z;
    groot_pose.position.x = -0.01565011581780207 + random_x;
    groot_pose.position.y = -0.019683543216663102 + random_y;
    groot_pose.position.z = 1.657396455658871 + random_z;

    float x_rotation = rad_distr(gen) * 0.01;
    float y_rotation = rad_distr(gen) * 0.01;
    float z_rotation = rad_distr(gen) * 0.01;

    rocket_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                rocket_q;

    groot_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                groot_q;

    rocket_pose.orientation.w = rocket_q.w();
    rocket_pose.orientation.x = rocket_q.x();
    rocket_pose.orientation.y = rocket_q.y();
    rocket_pose.orientation.z = rocket_q.z();
    groot_pose.orientation.w = groot_q.w();
    groot_pose.orientation.x = groot_q.x();
    groot_pose.orientation.y = groot_q.y();
    groot_pose.orientation.z = groot_q.z();

    dual_arm_group.clearPoseTargets();
    dual_arm_group.setStartStateToCurrentState();
    std::string rocket_eff = "rocket_tool0";
    dual_arm_group.setPoseTarget(rocket_pose, rocket_eff);

    std::string groot_eff = "groot_tool0";
    dual_arm_group.setPoseTarget(groot_pose, groot_eff);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (dual_arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      ROS_INFO("Plan did not successed");
    }
    dual_arm_group.execute(my_plan);
  }

  ros::shutdown();
}