#include "rclcpp/rclcpp.hpp"
#include "ros2_sim_motion_planner/motion_planner_node.hpp"
#include <Eigen/Geometry>

class MotionPlannerTester : public MotionPlannerNode
{
public:
  MotionPlannerTester() : MotionPlannerNode() {}

  void test()
  {

    // Test planToJointspacePosition method
    RCLCPP_INFO(this->get_logger(), "Testing planToJointspacePosition...");
    std::vector<double> joint_positions = {0.0, -1.3, 1.3, 0.0, 1.3, 0.0};
    auto position_trajectory = planToJointspacePosition(joint_positions);
    execute(position_trajectory);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Test planToCartesianPose PTP method
    RCLCPP_INFO(this->get_logger(), "Testing planToCartesianPose PTP...");
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translation() = Eigen::Vector3d(0.4, 0.0, 0.3);
    auto point_trajectory = planToCartesianPose(target_pose);
    execute(point_trajectory);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Test planToCartesianPose LIN method
    RCLCPP_INFO(this->get_logger(), "Testing planToCartesianPose LIN...");
    Eigen::Isometry3d lin_target_pose = Eigen::Isometry3d::Identity();
    lin_target_pose.translation() = Eigen::Vector3d(0.3, 0.1, 0.3);
    auto lin_trajectory = planToCartesianPose(lin_target_pose, "pilz_industrial_motion_planner", "LIN");
    execute(lin_trajectory);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlannerTester>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(),"Starting rclcpp spinner...");
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();
  node->initialize();
  node->test();
  rclcpp::shutdown();
  return 0;
}
