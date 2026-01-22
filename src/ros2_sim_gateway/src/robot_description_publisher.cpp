#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotDescriptionPublisher : public rclcpp::Node
{
public:
    RobotDescriptionPublisher()
        : Node("robot_description_publisher")
    {
        this->declare_parameter<std::string>("robot_description_path", "");
        const auto path = this->get_parameter("robot_description_path").as_string();

        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "robot_description", rclcpp::QoS(1).transient_local().reliable());

        if (path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "robot_description_path is empty. No URDF published.");
            return;
        }

        std::ifstream file(path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", path.c_str());
            return;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std_msgs::msg::String msg;
        msg.data = buffer.str();
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published robot description from %s", path.c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDescriptionPublisher>());
    rclcpp::shutdown();
    return 0;
}
