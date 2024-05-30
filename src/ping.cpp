#include <iostream>
#include <chrono>
#include <algorithm>
#include <atomic>
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
            this->declare_parameter("rate", 10);
            this->warmup_ = this->get_parameter("warmup").as_double();
            this->samples_ = this->get_parameter("samples").as_int();
            this->payload_size_ = this->get_parameter("size").as_int();
            this->rate_ = this->get_parameter("rate").as_int();
            std::cout << "Warm up time (sec): " << this->warmup_ << std::endl;
            std::cout << "Samples number: " << this->samples_ << std::endl;
            std::cout << "Pyaload size (bytes): " << this->payload_size_ << std::endl;
            std::cout << "Publish rate (Hz): " << this->rate_ << std::endl;

            // Create the message
            message_ = std::make_shared<simple_performance::msg::PingPong>();
            message_->data.resize(this->payload_size_);
            std::fill(message_->data.begin(), message_->data.end(), 0);

            // Init 
            this->start_time_ = high_resolution_clock::now();
            this->total_expected_time_ = this->warmup_ + ((float)this->samples_ / (float)this->rate_);

            // TODO: Able to configure QoS (reliability, history, durability)
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            ping_publisher_ = this->create_publisher<simple_performance::msg::PingPong>("ping", qos);
            pong_subscriber_ = this->create_subscription<simple_performance::msg::PingPong>(
                "pong", qos, std::bind(&Ping::recv_pong_callback, this, _1));
            timer_ = this->create_wall_timer(milliseconds(std::clamp(1000/this->rate_, 0, 1000)), std::bind(&Ping::timer_callback, this));
        }
        
        bool is_stop() {
            time_point<high_resolution_clock> now = high_resolution_clock::now();
            return (this->samples_idx_ >= this->samples_) && 
                    (now - this->start_time_ > duration<double, std::milli>((this->total_expected_time_)  * 1000));
        }
  
    private:
        void timer_callback() {
            if (this->samples_idx_ < this->samples_) {
                time_point<high_resolution_clock> t1 = high_resolution_clock::now();
                // Put the start time inside the message
                uint8_t *ptr = (uint8_t *)&t1;
                for (unsigned long i = 0; i < sizeof(t1); i++) {
                    message_->data[i] = *(ptr + i); 
                }
                ping_publisher_->publish(*message_);
                // Start to calculate after warmup
                if (t1 - this->start_time_ > duration<double, std::milli>(this->warmup_ * 1000)) {
                    this->samples_idx_++;
                }
            }
        }

        void recv_pong_callback(const simple_performance::msg::PingPong::SharedPtr msg) {
            time_point<high_resolution_clock> t2 = high_resolution_clock::now();
            // Retrieve the start time from the message
            time_point<high_resolution_clock> t1;
            uint8_t *ptr = (uint8_t *)&t1;
            for (unsigned long i = 0; i < sizeof(t1); i++) {
                *(ptr + i) = msg->data[i]; 
            }
            // Get RTT and RTT/2
            auto rtt = duration<double, std::micro>(t2 - t1).count();
            // Start to record after warmup
            if (t2 - this->start_time_ > duration<double, std::milli>(this->warmup_ * 1000)) {
                RCLCPP_INFO(this->get_logger(), "RTT: %lf(us), RTT/2: %lf(us)", rtt, rtt/2);
                this->result_.push_back(rtt);
            }
        }
  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<simple_performance::msg::PingPong>::SharedPtr ping_publisher_;
        rclcpp::Subscription<simple_performance::msg::PingPong>::SharedPtr pong_subscriber_;
        size_t samples_;
        std::atomic<std::size_t> samples_idx_;
        size_t payload_size_;
        float warmup_;
        float total_expected_time_;
        int rate_;
        simple_performance::msg::PingPong::SharedPtr message_;
        std::vector<double> result_;
        time_point<high_resolution_clock> start_time_;
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
