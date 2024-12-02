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
#include "../include/krabi_can_broker/can_structs.h"
#include "rclcpp/rclcpp.hpp"
#include <krabi_msgs/msg/actuators2025.hpp>


// Node class
class CanPublisherNode: public rclcpp::Node 
{
public:
    CanPublisherNode() : Node("can_publisher_node")
		     {
        // Initialize the CAN socket
        can_socket_ = init_can_socket("can0");

        // Create subscribers for each message type
        actuators2025_sub_ = this->create_subscription<krabi_msgs::msg::Actuators2025>(
            "actuators2025", 10, std::bind(&CanPublisherNode::servoCallback, this, std::placeholders::_1));

        /*cmd_vel_sub_ = this->create_subscription<CmdVel>(
            "cmd_vel", 10, std::bind(&CanPublisherNode::cmdVelCallback, this, std::placeholders::_1));*/

        // Additional subscribers for other message types...
    }

    ~CanPublisherNode() {
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

private:
    int can_socket_;

    // Servo callback
    void servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg) {
        struct can_frame frame;
        frame.can_id = can_ids::SERVO_1;
        frame.can_dlc = sizeof(ServoMessage);
	// @TODO complete this
        //memcpy(frame.data, msg.get(), sizeof(ServoMessage));
        send_can_frame(frame);
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
    auto node = std::make_shared<CanPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    /*auto canTest = CanPublisherNode();
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
