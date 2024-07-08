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
  using PlanAndExecute_Goal = ros2_sim_msgs::action::PlanAndExecute_Goal;
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
    if (goal->target_type == PlanAndExecute_Goal::CARTESIAN)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with Cartesian pose [%.2f, %.2f, %.2f]",
                  goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
    }
    else if (goal->target_type == PlanAndExecute_Goal::JOINT_SPACE)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with joint positions");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Received goal request with unknown target type");
      return rclcpp_action::GoalResponse::REJECT;
    }
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

    motion_planner_->setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);
    motion_planner_->setMaxAccelerationScalingFactor(goal->max_acceleration_scaling_factor);

    moveit_msgs::msg::RobotTrajectory::SharedPtr trajectory = nullptr;
    auto start = std::chrono::steady_clock::now();

    if (goal->target_type == PlanAndExecute_Goal::JOINT_SPACE)
    {
      while ((trajectory = motion_planner_->planToJointspacePosition(std::vector<double>(goal->joint_positions.begin(), goal->joint_positions.end()))) == nullptr
             && std::chrono::steady_clock::now() - start < std::chrono::duration<float>(goal->timeout))
      {
          feedback->progress = "planning";
          goal_handle->publish_feedback(feedback);
      }
    }
    else if (goal->target_type == PlanAndExecute_Goal::CARTESIAN)
    {
      Eigen::Isometry3d target_pose;
      tf2::fromMsg(goal->target_pose, target_pose);
      while ((trajectory = motion_planner_->planToCartesianPose(target_pose, goal->planning_pipeline, goal->planner_id)) == nullptr
             && std::chrono::steady_clock::now() - start < std::chrono::duration<float>(goal->timeout))
      {
          feedback->progress = "planning";
          goal_handle->publish_feedback(feedback);
      }
    }

    if (trajectory == nullptr)
    {
        result->success = false;
        result->message = "Trajectory planning failed";
        goal_handle->abort(result);
        return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

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
