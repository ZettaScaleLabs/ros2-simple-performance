#include "rclcpp/rclcpp.hpp"
// TODO: Use array able to set size
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Pong : public rclcpp::Node
{
    public:
        Pong() : Node("pong_node") {
            // TODO: Able to configure QoS
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "ping", qos, std::bind(&Pong::topic_callback, this, _1));
            pong_publisher_ = this->create_publisher<std_msgs::msg::String>("pong", qos);
        }
  
    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            auto message = *msg;
            pong_publisher_->publish(message);
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ping_subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pong_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pong>());
    rclcpp::shutdown();
    return 0;
}
