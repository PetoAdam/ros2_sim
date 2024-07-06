#ifndef MOTION_PLANNER_NODE_HPP_
#define MOTION_PLANNER_NODE_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ros2_sim_motion_planner/imotion_planner.hpp"

class MotionPlannerNode : public rclcpp::Node, public IMotionPlanner
{
public:
  MotionPlannerNode() 
    : rclcpp::Node("motion_planner")
  {
  }

  void initialize() override
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

    move_group_interface_->setMaxVelocityScalingFactor(0.2);
    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToCartesianPose(
    const Eigen::Isometry3d& pose,
    const std::string& planning_pipeline = "pilz_industrial_motion_planner",
    const std::string& planner_id = "PTP") override
  {
    setupPlanner(planning_pipeline, planner_id);
    move_group_interface_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!move_group_interface_->plan(plan)) {
      RCLCPP_ERROR(logger_, "Planning failed");
      return nullptr;
    }

    RCLCPP_INFO(logger_, "Planning successful");
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory);
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToJointspacePosition(
    const std::vector<double>& joint_positions) override
  {
    move_group_interface_->setJointValueTarget(joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!move_group_interface_->plan(plan)) {
      RCLCPP_ERROR(logger_, "Planning failed");
      return nullptr;
    }

    RCLCPP_INFO(logger_, "Planning successful");
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory);
  }

  bool execute(std::shared_ptr<moveit_msgs::msg::RobotTrajectory> trajectory) override
  {
    if (trajectory == nullptr)
    {
      RCLCPP_ERROR(logger_, "Trajectory invalid");
      return false;
    }

    RCLCPP_INFO(logger_, "Executing trajectory...");
    auto result = move_group_interface_->execute(*trajectory);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Trajectory execution failed...");
    }

    RCLCPP_INFO(logger_, "Trajectory successfully executed");
    return true;
  }

private:
  void setupPlanner(const std::string& planning_pipeline, const std::string& planner_id)
  {
    move_group_interface_->setPlanningPipelineId(planning_pipeline);
    move_group_interface_->setPlannerId(planner_id);
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Logger logger_ = rclcpp::get_logger("motion_planner");
  const std::string PLANNING_GROUP = "robot_arm";
};

#endif  // MOTION_PLANNER_NODE_HPP_
