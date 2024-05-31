#pragma once

#include "rclcpp/rclcpp.hpp"

class QoS
{
    public:
        rclcpp::QoS getQoS() {
            rclcpp::QoS qos(rclcpp::KeepLast{16});
            // Reliability
            if (this->reliability_ == "BEST_EFFORT") {
                qos.best_effort();
            } else if (this->reliability_ == "RELIABLE") {
                qos.reliable();
            } else {
                throw std::runtime_error("reliability should be either BEST_EFFORT / RELIABLE");
            }
            // History
            if (this->history_ == "KEEP_ALL") {
                qos.keep_all();
            } else if (this->history_ == "KEEP_LAST") {
                qos.keep_last(this->history_depth_);
            } else {
                throw std::runtime_error("history should be either KEEP_ALL / KEEP_LAST");
            }
            // Durability
            if (this->durability_ == "VOLATILE") {
                qos.durability_volatile();
            } else if (this->durability_ == "TRANSIENT_LOCAL") {
                qos.transient_local();
            } else {
                throw std::runtime_error("durability should be either VOLATILE / TRANSIENT_LOCAL");
            }
            return qos;
        }

        void printQoS() {
            std::cout << "QoS: " << this->reliability_ << "," << this->durability_ << "," << this-> history_;
            if (this->history_ == "KEEP_LAST") {
                std::cout << "(" << this->history_depth_ << ")" << std::endl;
            } else {
                std::cout << std::endl;
            }
        }

    protected:
        // QoS settings
        std::string reliability_;
        std::string history_;
        unsigned int history_depth_;
        std::string durability_;

};
