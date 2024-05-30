#include "rclcpp/rclcpp.hpp"
// TODO: Use array able to set size
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Ping : public rclcpp::Node
{
    public:
        Ping() : Node("ping_node") {
            // Get parameters
            this->declare_parameter("warmup", 15.0);
            this->declare_parameter("samples", 10);
            this->declare_parameter("size", 32);
            this->warmup_ = this->get_parameter("warmup").as_double();
            this->samples_ = this->get_parameter("samples").as_int();
            this->payload_size_ = this->get_parameter("size").as_int();
            std::cout << "Warm up time (sec): " << this->warmup_ << std::endl;
            std::cout << "Samples number: " << this->samples_ << std::endl;
            std::cout << "Pyaload size (bytes): " << this->payload_size_ << std::endl;

            // TODO: Support warmup
            // TODO: Able to configure QoS (reliability, history, durability)
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_publisher_ = this->create_publisher<std_msgs::msg::String>("ping", qos);
            pong_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "pong", qos, std::bind(&Ping::topic_callback, this, _1));
            timer_ = this->create_wall_timer(500ms, std::bind(&Ping::timer_callback, this));
        }
  
    private:
        void timer_callback() {
            // TODO: Add timestamp
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! Ping";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            ping_publisher_->publish(message);
        }
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            // TODO: Receive timestamp and calculate RTT
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pong_subscriber_;
        size_t samples_;
        size_t payload_size_;
        float warmup_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ping>());
    // TODO: Able to get the summary
    rclcpp::shutdown();
    return 0;
}
