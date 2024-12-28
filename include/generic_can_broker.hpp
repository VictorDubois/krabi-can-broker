#pragma once

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


// Node class
class GenericCanBroker: public rclcpp::Node 
{
public:
    GenericCanBroker();
    ~GenericCanBroker();
    void send_can_frame(const struct can_frame &frame);
    int init_can_socket(const std::string &interface);

protected:
    int can_socket_;
    std::thread can_receiver_thread_;

    void receive_can_messages();

};
