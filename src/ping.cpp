#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "simple_performance/msg/ping_pong.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Ping : public rclcpp::Node
{
    public:
        Ping() : Node("ping_node"), samples_idx_(0) {
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
                "pong", qos, std::bind(&Ping::recv_pong_callback, this, _1));
            timer_ = this->create_wall_timer(500ms, std::bind(&Ping::timer_callback, this));
        }
        
        bool is_stop() {
            return this->result_.size() >= this->samples_;
        }
  
    private:
        void timer_callback() {
            if (this->samples_idx_ < this->samples_) {
                time_point<high_resolution_clock> start = high_resolution_clock::now();
                // Put the start time inside the message
                uint8_t *ptr = (uint8_t *)&start;
                for (unsigned long i = 0; i < sizeof(start); i++) {
                    message_->data[i] = *(ptr + i); 
                }
                ping_publisher_->publish(*message_);
                this->samples_idx_++;
            }
        }

        void recv_pong_callback(const simple_performance::msg::PingPong::SharedPtr msg) {
            time_point<high_resolution_clock> stop = high_resolution_clock::now();
            // Retrieve the start time from the message
            time_point<high_resolution_clock> start;
            uint8_t *ptr = (uint8_t *)&start;
            for (unsigned long i = 0; i < sizeof(start); i++) {
                *(ptr + i) = msg->data[i]; 
            }
            // Get RTT and RTT/2
            auto rtt = duration<double, std::micro>(stop - start).count();
            RCLCPP_INFO(this->get_logger(), "RTT: %lf(us), RTT/2: %lf(us)", rtt, rtt/2);
            this->result_.push_back(rtt);
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<simple_performance::msg::PingPong>::SharedPtr ping_publisher_;
        rclcpp::Subscription<simple_performance::msg::PingPong>::SharedPtr pong_subscriber_;
        size_t samples_;
        size_t samples_idx_;
        size_t payload_size_;
        float warmup_;
        simple_performance::msg::PingPong::SharedPtr message_;
        std::vector<double> result_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto ping_node = std::make_shared<Ping>(); 
    while (!ping_node->is_stop()) {
        rclcpp::spin_some(ping_node);
    }
    // TODO: Able to get the summary
    rclcpp::shutdown();
    return 0;
}
