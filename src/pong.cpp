#include "rclcpp/rclcpp.hpp"
#include "simple_performance/msg/ping_pong.hpp"

using std::placeholders::_1;

class Pong : public rclcpp::Node
{
    public:
        Pong() : Node("pong_node") {
            // TODO: Able to configure QoS
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_subscriber_ = this->create_subscription<simple_performance::msg::PingPong>(
                "ping", qos, std::bind(&Pong::topic_callback, this, _1));
            pong_publisher_ = this->create_publisher<simple_performance::msg::PingPong>("pong", qos);
        }
  
    private:
        void topic_callback(const simple_performance::msg::PingPong::SharedPtr msg) const {
            pong_publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Receiving data size: %ld", msg->data.size());
        }
        rclcpp::Subscription<simple_performance::msg::PingPong>::SharedPtr ping_subscriber_;
        rclcpp::Publisher<simple_performance::msg::PingPong>::SharedPtr pong_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pong>());
    rclcpp::shutdown();
    return 0;
}
