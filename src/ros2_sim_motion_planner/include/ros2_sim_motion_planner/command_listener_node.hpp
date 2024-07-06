#ifndef COMMAND_LISTENER_NODE_HPP_
#define COMMAND_LISTENER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_sim_msgs/action/plan_and_execute.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "ros2_sim_motion_planner/motion_planner_node.hpp"

class CommandListenerNode : public rclcpp::Node
{
public:
  using PlanAndExecute = ros2_sim_msgs::action::PlanAndExecute;
  using GoalHandlePlanAndExecute = rclcpp_action::ServerGoalHandle<PlanAndExecute>;

  CommandListenerNode()
    : Node("command_listener_node"),
      motion_planner_(std::make_shared<MotionPlannerNode>())
  {
    this->initialize();
  }

  void initialize()
  {
    motion_planner_->initialize();

    this->action_server_ = rclcpp_action::create_server<PlanAndExecute>(
      this,
      "plan_and_execute",
      std::bind(&CommandListenerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CommandListenerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&CommandListenerNode::handleAccepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlanAndExecute::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with pose [%.2f, %.2f, %.2f]",
                goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&CommandListenerNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanAndExecute::Feedback>();
    auto result = std::make_shared<PlanAndExecute::Result>();

    // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(goal->target_pose, target_pose);

    // Plan the trajectory
    auto start = std::chrono::steady_clock::now();
    moveit_msgs::msg::RobotTrajectory::SharedPtr trajectory = nullptr;
    while ((trajectory = motion_planner_->planToCartesianPose(target_pose, goal->planning_pipeline, goal->planner_id)) == nullptr
           && std::chrono::steady_clock::now() - start < std::chrono::duration<float>(goal->timeout))
    {
        feedback->progress = "planning";
        goal_handle->publish_feedback(feedback);
    }

    if (trajectory == nullptr)
    {
        result->success = false;
        result->message = "Trajectory planning failed";
        goal_handle->abort(result);
        return;
    }

    // Check if goal is canceled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

    // Execute the trajectory
    feedback->progress = "executing";
    goal_handle->publish_feedback(feedback);
    if (trajectory && motion_planner_->execute(trajectory)) {
      result->success = true;
      result->message = "Trajectory successfully executed";
      goal_handle->succeed(result);
    } else {
      result->success = false;
      result->message = "Trajectory execution failed";
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<PlanAndExecute>::SharedPtr action_server_;
  std::shared_ptr<MotionPlannerNode> motion_planner_;
};

#endif  // COMMAND_LISTENER_NODE_HPP_
