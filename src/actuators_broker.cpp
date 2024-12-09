#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <cstring>
#include <string>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <net/if.h>      // for struct ifreq and IFNAMSIZ
#include <sys/ioctl.h>   // for ioctl and SIOCGIFINDEX
#include "../include/krabi_can_broker/CanStruct/can_structs.h"
#include "rclcpp/rclcpp.hpp"
#include <krabi_msgs/msg/actuators2025.hpp>
#include <sensor_msgs/msg/battery_state.hpp>


// Node class
class CanActuatorBroker: public rclcpp::Node 
{
public:
    CanActuatorBroker() : Node("can_publisher_node")
    {
        // Initialize the CAN socket
        can_socket_ = init_can_socket("can0");

        // Create subscribers for each message type
        actuators2025_sub_ = this->create_subscription<krabi_msgs::msg::Actuators2025>(
            "actuators2025", 10, std::bind(&CanActuatorBroker::servoCallback, this, std::placeholders::_1));

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("actuators_battery", 10);


        // Start a separate thread to listen to CAN messages
        can_receiver_thread_ = std::thread(&CanActuatorBroker::receive_can_messages, this);



        /*cmd_vel_sub_ = this->create_subscription<CmdVel>(
            "cmd_vel", 10, std::bind(&CanActuatorBroker::cmdVelCallback, this, std::placeholders::_1));*/

        // Additional subscribers for other message types...
    }

    ~CanActuatorBroker() {
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

private:
    int can_socket_;
    std::thread can_receiver_thread_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;


    // Servo callback
    void servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg) {
    struct can_frame frame;
    frame.can_id = can_ids::SERVO_1;
    frame.can_dlc = sizeof(ServoMessage);
    frame.data[0] = msg->servo_1.angle;
    frame.data[1] = msg->servo_1.speed;
    frame.data[2] = msg->servo_2.angle;
    frame.data[3] = msg->servo_2.speed;
    frame.data[4] = msg->servo_3.angle;
    frame.data[5] = msg->servo_3.speed;
    frame.data[6] = msg->servo_4.angle;
    frame.data[7] = msg->servo_4.speed;
    send_can_frame(frame);

    frame.can_id = can_ids::SERVO_2;
    frame.can_dlc = sizeof(ServoMessage);
    frame.data[0] = msg->servo_5.angle;
    frame.data[1] = msg->servo_5.speed;
    frame.data[2] = msg->servo_6.angle;
    frame.data[3] = msg->servo_6.speed;
    frame.data[4] = msg->servo_7.angle;
    frame.data[5] = msg->servo_7.speed;
    frame.data[6] = msg->servo_8.angle;
    frame.data[7] = msg->servo_8.speed;
    send_can_frame(frame);

    frame.can_id = can_ids::SCORE;
    frame.can_dlc = sizeof(Score);
    for (int i = 0; i < 8; i++)
    {
        frame.data[i] = 0;
    }
    frame.data[0] = msg->score;
        send_can_frame(frame);
    }

    void receive_can_messages() {
        struct can_frame frame;
        while (rclcpp::ok()) {
            int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "CAN read error");
                continue;
            }

            if (frame.can_id == ANALOG_SENSORS && frame.can_dlc == sizeof(AnalogSensors)) {
                //AnalogSensors analog_data;
                //std::memcpy(&analog_data, frame.data, sizeof(AnalogSensors));
                //publish_analog_sensors(analog_data);
                uint16_t bat_mV = frame.data[1] | (frame.data[0] << 8);
                publish_analog_sensors(bat_mV);
        
            }
        }
    }

    void publish_analog_sensors(const uint16_t &battery_voltage_mV) {
        // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
        auto msg = sensor_msgs::msg::BatteryState();
        msg.voltage = battery_voltage_mV / 1000.0; // Convert mV to V
        msg.present = true;

        RCLCPP_INFO(this->get_logger(), "Publishing AnalogSensors: battery_mV=%d", battery_voltage_mV);

        battery_pub_->publish(msg);
    }

    // CmdVel callback
    /*void cmdVelCallback(const CmdVel::SharedPtr msg) {
        struct can_frame frame;
        frame.can_id = CMD_VEL;
        frame.can_dlc = sizeof(CmdVel);
        memcpy(frame.data, msg.get(), sizeof(CmdVel));
        send_can_frame(frame);
    }*/

public:
    void send_can_frame(const struct can_frame &frame) {
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
        std::cout <<  "Failed to send CAN frame" << std::endl;
        }
    }

    int init_can_socket(const std::string &interface) {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
        std::cout <<  "Error while opening CAN socket" << std::endl;
            return -1;
        }
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
        ioctl(s, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
        std::cout <<  "Error in socket bind" << std::endl;
            return -1;
        }
        return s;
    }

    // Subscribers
    rclcpp::Subscription<krabi_msgs::msg::Actuators2025>::SharedPtr actuators2025_sub_;
    //rclcpp::Subscription<CmdVel>::SharedPtr cmd_vel_sub_;
    // Add other subscribers as needed...
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanActuatorBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    /*auto canTest = CanActuatorBroker();
    can_frame frame;
    frame.can_id = 10;
    frame.data[0] = 10;
    frame.data[1] = 99;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    frame.can_dlc = 8;
    // = {0, 99, 0, 0, 0, 0, 0, 0};
    canTest.send_can_frame(frame);
    */
    return 0;
}
