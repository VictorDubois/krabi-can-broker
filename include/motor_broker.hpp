#pragma once

#include "generic_can_broker.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <krabi_msgs/msg/motors.hpp>
#include <krabi_msgs/msg/motors_cmd.hpp>
#include <krabi_msgs/msg/motors_parameters.hpp>
#include <krabi_msgs/msg/odom_lighter.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

// Node class
class MotorBroker : public GenericCanBroker
{
public:
    MotorBroker();

    ~MotorBroker();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
    krabi_msgs::msg::OdomLighter odom_lighter_msg;

    rclcpp::Publisher<krabi_msgs::msg::OdomLighter>::SharedPtr odom_lighter_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    // rclcpp::Subscription<krabi_msgs::msg::Motors>::SharedPtr motors_sub_;
    void publish_analog_sensors(const int16_t& battery_voltage_mV);

    void receive_can_messages();

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void motorsParametersCallback(const krabi_msgs::msg::MotorsParameters::SharedPtr msg);
    void motorsCmdCallback(const krabi_msgs::msg::MotorsCmd::SharedPtr msg);
    void motorsEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsCmd>::SharedPtr motors_cmd_sub_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsParameters>::SharedPtr motors_parameters_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_enable_sub_;
};
