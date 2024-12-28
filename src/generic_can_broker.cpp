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
#include "../include/generic_can_broker.hpp"


GenericCanBroker::GenericCanBroker() : Node("can_publisher_node")
{
    // Initialize the CAN socket
    can_socket_ = init_can_socket("can0");

    // Start a separate thread to listen to CAN messages
    can_receiver_thread_ = std::thread(&GenericCanBroker::receive_can_messages, this);
}

GenericCanBroker::~GenericCanBroker() {
    if (can_socket_ != -1) {
        close(can_socket_);
    }
}



void GenericCanBroker::receive_can_messages() {
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN read error");
            continue;
        }
    }
}

void GenericCanBroker::send_can_frame(const struct can_frame &frame) {
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
    std::cout <<  "Failed to send CAN frame" << std::endl;
    }
}

int GenericCanBroker::init_can_socket(const std::string &interface) {
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


/*int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericCanBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/
