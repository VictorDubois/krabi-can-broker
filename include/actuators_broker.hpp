#pragma once

#include "generic_can_broker.hpp"
#include <krabi_msgs/msg/actuators2025.hpp>
#include <krabi_msgs/msg/infos_stepper.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>


// Node class
class CanActuatorBroker: public GenericCanBroker
{
public:
    CanActuatorBroker();

    ~CanActuatorBroker() {    
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    };

private:
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<krabi_msgs::msg::InfosStepper>::SharedPtr stepper_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vacuum_pub_;
    
    // Servo callback
    void servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg) ;

    void receive_can_messages();
    void publish_analog_sensors(const int16_t &battery_voltage_mV) ;
    void publish_vacuum(const int16_t &stepper_info) ;

    void publish_stepper_info(const CAN::StepperInfo* stepper_info) ;
    rclcpp::Subscription<krabi_msgs::msg::Actuators2025>::SharedPtr actuators2025_sub_;

};
