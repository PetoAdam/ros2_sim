#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <jsoncpp/json/json.h>
#include <sstream>

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

        robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&JointStatesListener::robotDescriptionCallback, this, std::placeholders::_1));

        ws_server_.init_asio();
        ws_server_.set_reuse_addr(true);
        ws_server_.clear_access_channels(websocketpp::log::alevel::frame_header | websocketpp::log::alevel::frame_payload);
        ws_server_.set_open_handler(std::bind(&JointStatesListener::on_open, this, std::placeholders::_1));
        ws_server_.set_close_handler(std::bind(&JointStatesListener::on_close, this, std::placeholders::_1));
        ws_server_.set_message_handler(std::bind(&JointStatesListener::on_message, this, std::placeholders::_1, std::placeholders::_2));

        ws_thread_ = std::thread([this]() {
            ws_server_.listen(boost::asio::ip::tcp::v4(), 1235);
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
    bool sendMessage(connection_hdl hdl, const std::string &message)
    {
        websocketpp::lib::error_code ec;
        auto conn = ws_server_.get_con_from_hdl(hdl, ec);
        if (ec)
        {
            return false;
        }

        if (conn->get_state() != websocketpp::session::state::open)
        {
            return false;
        }

        ws_server_.send(hdl, message, websocketpp::frame::opcode::text, ec);
        if (ec)
        {
            return false;
        }

        return true;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Json::Value root;
        root["type"] = "joint_states";
        root["names"] = Json::Value(Json::arrayValue);
        root["positions"] = Json::Value(Json::arrayValue);
        for (const auto &name : msg->name)
        {
            root["names"].append(name);
        }
        for (const auto &position : msg->position)
        {
            root["positions"].append(position);
        }

        Json::StreamWriterBuilder writer;
        const std::string json_message = Json::writeString(writer, root);

        for (auto it = connections_.begin(); it != connections_.end();)
        {
            if (!sendMessage(*it, json_message))
            {
                it = connections_.erase(it);
                continue;
            }
            ++it;
        }
    }

    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_robot_description_ = msg->data;

        Json::Value root;
        root["type"] = "robot_description";
        root["urdf"] = msg->data;

        Json::StreamWriterBuilder writer;
        const std::string json_message = Json::writeString(writer, root);

        for (auto it = connections_.begin(); it != connections_.end();)
        {
            if (!sendMessage(*it, json_message))
            {
                it = connections_.erase(it);
                continue;
            }
            ++it;
        }
    }

    void on_open(connection_hdl hdl)
    {
        connections_.insert(hdl);

        if (!last_robot_description_.empty())
        {
            Json::Value root;
            root["type"] = "robot_description";
            root["urdf"] = last_robot_description_;

            Json::StreamWriterBuilder writer;
            const std::string json_message = Json::writeString(writer, root);
            sendMessage(hdl, json_message);
        }
    }

    void on_close(connection_hdl hdl)
    {
        connections_.erase(hdl);
    }

    void on_message(connection_hdl hdl, server::message_ptr msg)
    {
        const auto payload = msg->get_payload();
        RCLCPP_INFO(this->get_logger(), "Received message: %s", payload.c_str());

        Json::Value request;
        Json::CharReaderBuilder reader_builder;
        std::string errors;
        std::istringstream stream(payload);

        if (Json::parseFromStream(reader_builder, stream, &request, &errors))
        {
            if (request.isMember("type") && request["type"].asString() == "request_robot_description")
            {
                if (!last_robot_description_.empty())
                {
                    Json::Value root;
                    root["type"] = "robot_description";
                    root["urdf"] = last_robot_description_;

                    Json::StreamWriterBuilder writer;
                    const std::string json_message = Json::writeString(writer, root);
                    sendMessage(hdl, json_message);
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
    std::string last_robot_description_;
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
