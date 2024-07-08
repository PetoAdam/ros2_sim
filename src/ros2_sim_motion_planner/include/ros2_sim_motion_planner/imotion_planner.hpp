#ifndef IMOTION_PLANNER_HPP_
#define IMOTION_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

class IMotionPlanner
{
public:
  virtual ~IMotionPlanner() = default;

  virtual void initialize() = 0;

  virtual moveit_msgs::msg::RobotTrajectory::SharedPtr planToCartesianPose(
    const Eigen::Isometry3d& pose,
    const std::string& planning_pipeline = "pilz_industrial_motion_planner",
    const std::string& planner_id = "PTP") = 0;

  virtual moveit_msgs::msg::RobotTrajectory::SharedPtr planToJointspacePosition(
    const std::vector<double>& joint_positions) = 0;

  virtual bool execute(std::shared_ptr<moveit_msgs::msg::RobotTrajectory> trajectory) = 0;
};

#endif  // IMOTION_PLANNER_HPP_
