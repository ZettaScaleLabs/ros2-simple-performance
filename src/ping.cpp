#include "rclcpp/rclcpp.hpp"
// TODO: Use array able to set size
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Ping : public rclcpp::Node
{
    public:
        Ping() : Node("ping_node"), count_(0) {
            // TODO: Support warmup
            // TODO: Able to configure QoS
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_publisher = this->create_publisher<std_msgs::msg::String>("ping", qos);
            pong_subscriber = this->create_subscription<std_msgs::msg::String>(
                "pong", qos, std::bind(&Ping::topic_callback, this, _1));
            timer_ = this->create_wall_timer(500ms, std::bind(&Ping::timer_callback, this));
        }
  
    private:
        void timer_callback() {
            // TODO: Add timestamp
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            ping_publisher->publish(message);
        }
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            // TODO: Receive timestamp and calculate RTT
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pong_subscriber;
        size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ping>());
    // TODO: Able to get the summary
    rclcpp::shutdown();
    return 0;
}
