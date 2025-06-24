#pragma once

#include "generic_can_broker.hpp"
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <krabi_msgs/msg/actuators2025.hpp>
#include <krabi_msgs/msg/ax12_info.hpp>
#include <krabi_msgs/msg/infos_stepper.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/float32.hpp>

// Node class
class CanActuatorBroker : public GenericCanBroker
{
public:
    CanActuatorBroker();

    ~CanActuatorBroker()
    {
        if (can_socket_ != -1)
        {
            close(can_socket_);
        }
    };

    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
    bool m_is_blue;
    int8_t m_remaining_time = 100;
    CAN::Obstacles m_current_obstacles;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_power_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_elec_pub_;
    rclcpp::Publisher<krabi_msgs::msg::InfosStepper>::SharedPtr stepper_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vacuum_pub_;
    rclcpp::Publisher<krabi_msgs::msg::AX12Info>::SharedPtr AX12_pub_1;
    rclcpp::Publisher<krabi_msgs::msg::AX12Info>::SharedPtr AX12_pub_2;
    rclcpp::Publisher<krabi_msgs::msg::AX12Info>::SharedPtr AX12_pub_3;
    rclcpp::Publisher<krabi_msgs::msg::AX12Info>::SharedPtr AX12_pub_4;
    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr digitalReads_pub;
    // Servo callback
    void servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg);
    void remainingTimeCallback(const builtin_interfaces::msg::Duration::SharedPtr msg);
    void isBlueCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void updateLidarCallback(
      std::shared_ptr<geometry_msgs::msg::PoseStamped const> closest_obstacle,
      bool front);
    void sendAX12Write(const krabi_msgs::msg::AX12Cmd msg, CAN::can_ids id);

    void receive_can_messages();

    void publish_analog_sensors(const int16_t& battery_power_mV, const int16_t& battery_elec_mV);
    void publish_vacuum(const int16_t& a_vacuum);
    void publish_digital_sensors(const uint8_t& a_digital_intputs);
    void publish_AX12(const CAN::AX12Read& ax12_read, int id);
    void publish_stepper_info(const CAN::StepperInfo* stepper_info);

    rclcpp::Subscription<krabi_msgs::msg::Actuators2025>::SharedPtr actuators2025_sub_;
    rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr remaining_time_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_blue_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_posestamped_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      obstacle_behind_posestamped_sub_;
};
