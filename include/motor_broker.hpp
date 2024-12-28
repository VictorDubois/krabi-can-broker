#pragma once

#include "generic_can_broker.hpp"
#include <krabi_msgs/msg/motors.hpp>
#include <krabi_msgs/msg/motors_cmd.hpp>
#include <krabi_msgs/msg/motors_parameters.hpp>
#include <krabi_msgs/msg/odom_lighter.hpp>

// Node class
class MotorBroker: public GenericCanBroker
{
public:
    MotorBroker();

    ~MotorBroker();

private:
    rclcpp::Publisher<krabi_msgs::msg::OdomLighter>::SharedPtr odom_lighter_pub_;

    void receive_can_messages();

    rclcpp::Subscription<krabi_msgs::msg::Motors>::SharedPtr motors_sub_;
};
