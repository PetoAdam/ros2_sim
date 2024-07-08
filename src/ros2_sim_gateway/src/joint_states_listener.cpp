#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <jsoncpp/json/json.h>

using websocketpp::connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;

class JointStatesListener : public rclcpp::Node
{
public:
    JointStatesListener()
        : Node("joint_states_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&JointStatesListener::jointStateCallback, this, std::placeholders::_1));

        ws_server_.init_asio();
        ws_server_.clear_access_channels(websocketpp::log::alevel::frame_header | websocketpp::log::alevel::frame_payload);
        ws_server_.set_open_handler(std::bind(&JointStatesListener::on_open, this, std::placeholders::_1));
        ws_server_.set_close_handler(std::bind(&JointStatesListener::on_close, this, std::placeholders::_1));
        ws_server_.set_message_handler(std::bind(&JointStatesListener::on_message, this, std::placeholders::_1, std::placeholders::_2));

        ws_thread_ = std::thread([this]() {
            ws_server_.listen(1235);
            ws_server_.start_accept();
            ws_server_.run();
        });

        RCLCPP_INFO(this->get_logger(), "WebSocket server started on ws://localhost:1235");
    }

    ~JointStatesListener()
    {
        ws_server_.stop_listening();
        ws_server_.stop();
        if (ws_thread_.joinable())
        {
            ws_thread_.join();
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Json::Value root;
        root["positions"] = Json::Value(Json::arrayValue);
        for (const auto &position : msg->position)
        {
            root["positions"].append(position);
        }

        Json::StreamWriterBuilder writer;
        const std::string json_message = Json::writeString(writer, root);

        for (const auto &hdl : connections_)
        {
            ws_server_.send(hdl, json_message, websocketpp::frame::opcode::text);
        }
    }

    void on_open(connection_hdl hdl)
    {
        connections_.insert(hdl);
    }

    void on_close(connection_hdl hdl)
    {
        connections_.erase(hdl);
    }

    void on_message(connection_hdl hdl, server::message_ptr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->get_payload().c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    server ws_server_;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;
    std::thread ws_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatesListener>());
    rclcpp::shutdown();
    return 0;
}
