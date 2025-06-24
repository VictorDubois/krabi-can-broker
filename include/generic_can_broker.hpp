#pragma once

#include "../include/krabi_can_broker/CanStruct/can_structs.h"
#include "rclcpp/rclcpp.hpp"
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h> // for struct ifreq and IFNAMSIZ
#include <string>
#include <sys/ioctl.h> // for ioctl and SIOCGIFINDEX
#include <sys/socket.h>
#include <unistd.h>

// Node class
class GenericCanBroker : public rclcpp::Node
{
public:
    GenericCanBroker();
    ~GenericCanBroker();
    void send_can_frame(const struct can_frame& frame);
    int init_can_socket(const std::string& interface);
    bool m_cannot_send_CAN_frame = false;
    bool m_cannot_open_CAN_socket = false;
    bool m_cannot_bind_CAN_socket = false;
    bool m_CAN_read_error = false;
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

protected:
    int can_socket_;
    std::thread can_receiver_thread_;

    void receive_can_messages();
};
