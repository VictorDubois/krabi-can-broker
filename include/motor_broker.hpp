#pragma once

#include "generic_can_broker.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
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
    krabi_msgs::msg::OdomLighter odom_lighter_msg;

    rclcpp::Publisher<krabi_msgs::msg::OdomLighter>::SharedPtr odom_lighter_pub_;
    //rclcpp::Subscription<krabi_msgs::msg::Motors>::SharedPtr motors_sub_;

    void receive_can_messages();

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) ;
    void motorsParametersCallback(const krabi_msgs::msg::MotorsParameters::SharedPtr msg) ;
    void motorsCmdCallback(const krabi_msgs::msg::MotorsCmd::SharedPtr msg) ;
    void motorsEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) ;

    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsCmd>::SharedPtr motors_cmd_sub_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsParameters>::SharedPtr motors_parameters_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_enable_sub_;
};
