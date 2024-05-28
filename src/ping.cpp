#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Ping : public rclcpp::Node
{
    public:
        Ping() : Node("ping_node"), count_(0) {
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            publisher_ = this->create_publisher<std_msgs::msg::String>("ping", qos);
            timer_ = this->create_wall_timer(500ms, std::bind(&Ping::timer_callback, this));
        }
  
    private:
        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ping>());
    rclcpp::shutdown();
    return 0;
}
