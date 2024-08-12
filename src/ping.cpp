#include <iostream>
#include <chrono>
#include <algorithm>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "simple_performance/msg/ping_pong.hpp"
#include "simple_performance/qos.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Ping : public rclcpp::Node, public QoS
{
    public:
        Ping() : Node("ping_node"), samples_idx_(0) {
            // Get parameters
            this->declare_parameter("warmup", 5.0);
            this->declare_parameter("samples", 100);
            this->declare_parameter("size", 32);
            this->declare_parameter("rate", 10);
            this->declare_parameter("reliability", "RELIABLE");
            this->declare_parameter("history", "KEEP_LAST");
            this->declare_parameter("history_depth", 16);
            this->declare_parameter("durability", "VOLATILE");
            this->reliability_ = this->get_parameter("reliability").as_string();
            this->history_ = this->get_parameter("history").as_string();
            this->history_depth_ = this->get_parameter("history_depth").as_int();
            this->durability_ = this->get_parameter("durability").as_string();
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

            // Configure QoS
            auto qos = this->getQoS();
            this->printQoS();
            ping_publisher_ = this->create_publisher<simple_performance::msg::PingPong>("ping", qos);
            pong_subscriber_ = this->create_subscription<simple_performance::msg::PingPong>(
                "pong", qos, std::bind(&Ping::recv_pong_callback, this, _1));
            timer_ = this->create_wall_timer(milliseconds(std::clamp(1000/this->rate_, 0, 1000)), std::bind(&Ping::timer_callback, this));
        }

        bool is_running() {
            if (this->samples_idx_ < this->samples_) {
                return true;
            }

            const auto now = high_resolution_clock::now();
            const auto elapsed_ms = duration_cast<milliseconds>(now - this->start_time_).count();
            if (elapsed_ms <= this->total_expected_time_) {
                return true;
            }

            return false;
        }

        void show_result() {
            if (this->result_.empty()) {
                std::cout << "[ERROR] Empty results." << std::endl;
                return;
            }

            const size_t N = this->result_.size();
            if (N > this->samples_) {
                std::cout << "[ERROR] Received " << N << " messages which should not be greater than the total samples " << this->samples_ << std::endl;
                return;
            }

            sort(this->result_.begin(), this->result_.end());

            // Disable the scientific notation globally
            std::cout << std::fixed;

            std::cout << "[RTT(us)] "
                << "min: " << this->result_[0] << ", "
                << "p05: " << this->result_[N * 0.05] << ", "
                << "p50: " << this->result_[N * 0.50] << ", "
                << "p95: " << this->result_[N * 0.95] << ", "
                << "max: " << this->result_[N - 1] << std::endl;

            std::cout << "[Loss(%)] " << (this->samples_ - N) * 100 / this->samples_ << std::endl;
        }

    private:
        void timer_callback() {
            if (this->samples_idx_ >= this->samples_) {
                return;
            }

            // Generate the timestamp
            const auto now = high_resolution_clock::now();
            uint64_t timestamp = duration_cast<microseconds>(now.time_since_epoch()).count();
            std::memmove(message_->data.data(), &timestamp, sizeof(timestamp));

            // Publish the message
            ping_publisher_->publish(*message_);

            // Start to calculate after warmup
            if (this->is_warmup_done()) {
                this->samples_idx_++;
            }
        }

        bool is_warmup_done() {
            // Check if it's still in warmup
            if (!this->warmup_done_) {
                const auto now = high_resolution_clock::now();
                this->warmup_done_ = duration_cast<milliseconds>(now - this->start_time_).count() > (this->warmup_ * 1000);
            }
            return this->warmup_done_;
        }

        void recv_pong_callback(const simple_performance::msg::PingPong::SharedPtr msg) {
            uint64_t pre_timestamp;
            std::memmove(&pre_timestamp, msg->data.data(), sizeof(pre_timestamp));

            const auto now = high_resolution_clock::now();
            uint64_t cur_timestamp = duration_cast<microseconds>(now.time_since_epoch()).count();
            // Get RTT and RTT/2 in microseconds
            auto rtt = cur_timestamp - pre_timestamp;


            // Start to record after warmup
            if (this->is_warmup_done()) {
                RCLCPP_INFO(this->get_logger(), "RTT: %ld(us), RTT/2: %ld(us)", rtt, rtt / 2);
                this->result_.push_back(rtt);
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<simple_performance::msg::PingPong>::SharedPtr ping_publisher_;
        rclcpp::Subscription<simple_performance::msg::PingPong>::SharedPtr pong_subscriber_;
        // Argument
        size_t samples_;
        size_t payload_size_;
        float warmup_;
        bool warmup_done_ = false;
        int rate_;
        // Internal usage
        float total_expected_time_;
        std::atomic<std::size_t> samples_idx_;
        simple_performance::msg::PingPong::SharedPtr message_;
        std::vector<double> result_;
        time_point<high_resolution_clock> start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto ping_node = std::make_shared<Ping>();
    while (ping_node->is_running()) {
        rclcpp::spin_some(ping_node);
    }
    ping_node->show_result();
    rclcpp::shutdown();
    return 0;
}
