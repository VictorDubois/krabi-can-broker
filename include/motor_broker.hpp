#pragma once

#include "generic_can_broker.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <krabi_msgs/msg/c620_dual_output.hpp>
#include <krabi_msgs/msg/c620_output.hpp>
#include <krabi_msgs/msg/motors.hpp>
#include <krabi_msgs/msg/motors_cmd.hpp>
#include <krabi_msgs/msg/motors_current.hpp>
#include <krabi_msgs/msg/motors_parameters.hpp>
#include <krabi_msgs/msg/odom_lighter.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

constexpr float C620_wheel_diameter = 0.060f; // 60mm // @todo update
constexpr float rpm_to_m_s_ratio = C620_wheel_diameter * M_PI / 60.f;
constexpr float C620_8192_ticks_to_deg_ratio = (360.f / 8192.f);
constexpr float centi_deg_to_rad = 1 / (100.0f * 180.f / M_PI);

// Node class
class MotorBroker : public GenericCanBroker
{
public:
    MotorBroker();

    ~MotorBroker();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
    krabi_msgs::msg::OdomLighter odom_lighter_msg;
    krabi_msgs::msg::MotorsCurrent motors_current_msg;
    krabi_msgs::msg::C620DualOutput C620Output_dual_msg;

    rclcpp::Publisher<krabi_msgs::msg::OdomLighter>::SharedPtr odom_lighter_pub_;
    rclcpp::Publisher<krabi_msgs::msg::MotorsCurrent>::SharedPtr motors_current_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<krabi_msgs::msg::C620DualOutput>::SharedPtr c620_pub_;
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

    uint16_t skip_the_next_C620_ouput_1_packets = 0;
    uint16_t skip_the_next_C620_ouput_2_packets = 0;
};
