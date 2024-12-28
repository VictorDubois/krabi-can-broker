#pragma once

#include "generic_can_broker.hpp"
#include <krabi_msgs/msg/actuators2025.hpp>
#include <krabi_msgs/msg/infos_stepper.hpp>
#include <sensor_msgs/msg/battery_state.hpp>


// Node class
class CanActuatorBroker: public GenericCanBroker
{
public:
    CanActuatorBroker();

    ~CanActuatorBroker();

private:
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<krabi_msgs::msg::InfosStepper>::SharedPtr stepper_info_pub_;

    // Servo callback
    void servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg) ;

    void receive_can_messages();
    void publish_analog_sensors(const uint16_t &battery_voltage_mV) ;

    void publish_stepper_info(const StepperInfo* stepper_info) ;
    rclcpp::Subscription<krabi_msgs::msg::Actuators2025>::SharedPtr actuators2025_sub_;

};
