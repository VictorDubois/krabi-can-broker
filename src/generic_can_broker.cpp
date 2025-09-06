#include "../include/generic_can_broker.hpp"
#include "../include/krabi_can_broker/CanStruct/can_structs.h"
#include "rclcpp/node.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h> // for struct ifreq and IFNAMSIZ
#include <string>
#include <sys/ioctl.h> // for ioctl and SIOCGIFINDEX
#include <sys/socket.h>
#include <unistd.h>

GenericCanBroker::GenericCanBroker()
  : Node("can_publisher_node")
{
    // Initialize the CAN socket
    can_socket_ = init_can_socket("can0");

    // Start a separate thread to listen to CAN messages
    // (to be done in children)
    // can_receiver_thread_ = std::thread(&GenericCanBroker::receive_can_messages, this);
}

GenericCanBroker::~GenericCanBroker()
{
    if (can_socket_ != -1)
    {
        close(can_socket_);
    }
}

void GenericCanBroker::receive_can_messages()
{
    struct can_frame frame;
    while (rclcpp::ok())
    {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN read error");
            m_CAN_read_error = true;
            continue;
        }
        m_CAN_read_error = false;
    }
}

void GenericCanBroker::send_can_frame(const struct can_frame& frame)
{
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
        std::cout << "Failed to send CAN frame" << std::endl;
        m_cannot_send_CAN_frame = true;
    }
    m_cannot_send_CAN_frame = false;
}

int GenericCanBroker::init_can_socket(const std::string& interface)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
        std::cout << "Error while opening CAN socket" << std::endl;
        m_cannot_open_CAN_socket = true;
        return -1;
    }
    m_cannot_open_CAN_socket = false;
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
        m_cannot_bind_CAN_socket = true;
        std::cout << "Error in socket bind" << std::endl;
        return -1;
    }
    m_cannot_bind_CAN_socket = false;
    return s;
}

void GenericCanBroker::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (m_cannot_send_CAN_frame)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unable to send CAN frame");
        stat.add("Unable to send CAN frame", 0);
    }
    if (m_cannot_open_CAN_socket)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unable to open CAN socket");
        stat.add("Unable to open CAN socket", 0);
    }
    if (m_cannot_bind_CAN_socket)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unable to bind CAN socket");
        stat.add("Unable to bind CAN socket", 0);
    }
    if (m_CAN_read_error)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unable to read CAN frame");
        stat.add("Unable to read CAN frame", 0);
    }
}

/*int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericCanBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/
