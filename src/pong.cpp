#include "rclcpp/rclcpp.hpp"
#include "simple_performance/msg/ping_pong.hpp"
#include "simple_performance/qos.hpp"

using std::placeholders::_1;

class Pong : public rclcpp::Node, public QoS
{
    public:
        Pong() : Node("pong_node") {
            this->declare_parameter("reliability", "RELIABLE");
            this->declare_parameter("history", "KEEP_LAST");
            this->declare_parameter("history_depth", 16);
            this->declare_parameter("durability", "VOLATILE");
            this->reliability_ = this->get_parameter("reliability").as_string();
            this->history_ = this->get_parameter("history").as_string();
            this->history_depth_ = this->get_parameter("history_depth").as_int();
            this->durability_ = this->get_parameter("durability").as_string();

            // Configure QoS
            auto qos = this->getQoS();
            this->printQoS();
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
