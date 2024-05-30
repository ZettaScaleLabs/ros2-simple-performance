#include "rclcpp/rclcpp.hpp"
#include "simple_performance/msg/ping_pong.hpp"

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

            // Create the message
            message_ = std::make_shared<simple_performance::msg::PingPong>();
            message_->data.resize(this->payload_size_);
            std::fill(message_->data.begin(), message_->data.end(), 0);

            // TODO: Support warmup
            // TODO: Able to configure QoS (reliability, history, durability)
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_publisher_ = this->create_publisher<simple_performance::msg::PingPong>("ping", qos);
            pong_subscriber_ = this->create_subscription<simple_performance::msg::PingPong>(
                "pong", qos, std::bind(&Ping::topic_callback, this, _1));
            timer_ = this->create_wall_timer(500ms, std::bind(&Ping::timer_callback, this));
        }
  
    private:
        void timer_callback() {
            // TODO: Add timestamp
            RCLCPP_INFO(this->get_logger(), "Publishing: size='%ld'", message_->data.size());
            ping_publisher_->publish(*message_);
        }
        void topic_callback(const simple_performance::msg::PingPong::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: size='%ld'", msg->data.size());
            // TODO: Receive timestamp and calculate RTT
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<simple_performance::msg::PingPong>::SharedPtr ping_publisher_;
        rclcpp::Subscription<simple_performance::msg::PingPong>::SharedPtr pong_subscriber_;
        size_t samples_;
        size_t payload_size_;
        float warmup_;
        simple_performance::msg::PingPong::SharedPtr message_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ping>());
    // TODO: Able to get the summary
    rclcpp::shutdown();
    return 0;
}
