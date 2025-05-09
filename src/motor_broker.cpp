#include "motor_broker.hpp"

MotorBroker::MotorBroker()
  : GenericCanBroker()
{
    // Initialize the CAN socket
    can_socket_ = init_can_socket("can0");

    // Create subscribers for each message type
    // motor_sub_ = this->create_subscription<krabi_msgs::msg::Motors>(
    //   "motor", 10, std::bind(&MotorBroker::motorCallback, this, std::placeholders::_1));

    odom_lighter_pub_ = this->create_publisher<krabi_msgs::msg::OdomLighter>("odom_lighter", 10);
    odom_lighter_msg = krabi_msgs::msg::OdomLighter();

    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("motor_battery", 10);

    // Start a separate thread to listen to CAN messages
    can_receiver_thread_ = std::thread(&MotorBroker::receive_can_messages, this);

    motors_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "enable_motor",
      10,
      std::bind(&MotorBroker::motorsEnableCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MotorBroker::cmdVelCallback, this, std::placeholders::_1));

    motors_cmd_sub_ = this->create_subscription<krabi_msgs::msg::MotorsCmd>(
      "motors_cmd", 10, std::bind(&MotorBroker::motorsCmdCallback, this, std::placeholders::_1));

    motors_parameters_sub_ = this->create_subscription<krabi_msgs::msg::MotorsParameters>(
      "motors_parameters",
      10,
      std::bind(&MotorBroker::motorsParametersCallback, this, std::placeholders::_1));

    // Additional subscribers for other message types...
}

MotorBroker::~MotorBroker()
{
    if (can_socket_ != -1)
    {
        close(can_socket_);
    }
}

void MotorBroker::receive_can_messages()
{
    struct can_frame frame;
    while (rclcpp::ok())
    {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN read error");
            continue;
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_XY_FLOAT
            && frame.can_dlc == sizeof(CAN::OdometryXYFloat))
        {
            // odom_lighter_msg.x = frame.data[3] | (frame.data[2] << 8) | (frame.data[1] << 16) |
            // (frame.data[0] << 24); odom_lighter_msg.y = frame.data[7] | (frame.data[6] << 8) |
            // (frame.data[5] << 16) | (frame.data[4] << 24);
            size_t l_float_length = sizeof(frame.data[0]) * 4;
            std::memcpy(&odom_lighter_msg.pose_x, frame.data, l_float_length);
            std::memcpy(&odom_lighter_msg.pose_y, frame.data + l_float_length, l_float_length);
            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_LIGHT
            && frame.can_dlc == sizeof(CAN::OdometryLight))
        {
            int32_t poseX_mm = frame.data[2] | (frame.data[1] << 8) | (frame.data[0] << 16);
            odom_lighter_msg.pose_x = poseX_mm / 1000.0f;

            int32_t poseY_mm = frame.data[5] | (frame.data[4] << 8) | (frame.data[3] << 16);
            odom_lighter_msg.pose_y = poseY_mm / 1000.0f;

            int16_t angleRz_centi_deg = frame.data[7] | (frame.data[6] << 8);
            odom_lighter_msg.angle_rz = angleRz_centi_deg / (100.0f * 180.f / M_PI);

            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_XY && frame.can_dlc == sizeof(CAN::OdometryXY))
        {
            int32_t poseX_mm = frame.data[3] | (frame.data[2] << 8) | (frame.data[1] << 16)
                               | (frame.data[0] << 24);
            odom_lighter_msg.pose_x = poseX_mm / 1000.0f;

            int32_t poseY_mm = frame.data[7] | (frame.data[6] << 8) | (frame.data[5] << 16)
                               | (frame.data[4] << 24);
            odom_lighter_msg.pose_y = poseY_mm / 1000.0f;

            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_THETA
            && frame.can_dlc == sizeof(CAN::OdometryThetaAndCurrent))
        {
            int32_t angleRz_centi_deg = frame.data[2] | (frame.data[2] << 8) | (frame.data[1] << 16)
                                        | (frame.data[0] << 24);

            int16_t current_left = frame.data[5] | (frame.data[4] << 8);
            int16_t current_right = frame.data[7] | (frame.data[6] << 8);
            odom_lighter_msg.angle_rz = angleRz_centi_deg / (100.0f * 180.f / M_PI);

            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_THETA_FLOAT
            && frame.can_dlc == sizeof(CAN::OdometryThetaFloat))
        {
            size_t l_float_length = sizeof(frame.data[0]) * 4;
            std::memcpy(&odom_lighter_msg.angle_rz, frame.data, l_float_length);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_SPEED_FLOAT
            && frame.can_dlc == sizeof(CAN::SpeedOdometryFloat))
        {
            size_t l_float_length = sizeof(frame.data[0]) * 4;
            std::memcpy(&odom_lighter_msg.speed_vx, frame.data, l_float_length);
            std::memcpy(&odom_lighter_msg.speed_wz, frame.data + l_float_length, l_float_length);
            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::ODOMETRY_SPEED
            && frame.can_dlc == sizeof(CAN::SpeedOdometry))
        {
            int32_t speedVx_µm_s = (frame.data[0] << 24) | (frame.data[1] << 16)
                                   | (frame.data[2] << 8) | frame.data[3];
            odom_lighter_msg.speed_vx = speedVx_µm_s / 1000000.f;

            int32_t speedWz_mrad_s = (frame.data[4] << 24) | (frame.data[5] << 16)
                                     | (frame.data[6] << 8) | frame.data[7];
            odom_lighter_msg.speed_wz = speedWz_mrad_s / 1000.f;
            odom_lighter_pub_->publish(odom_lighter_msg);
        }

        if (frame.can_id == CAN::can_ids::MOTOR_BOARD_BATTERY
            && frame.can_dlc == sizeof(CAN::AnalogSensors))
        {
            CAN::AnalogSensors battery_info;
            battery_info.battery_mV = frame.data[1] | (frame.data[0] << 8);
            publish_analog_sensors(battery_info.battery_mV);
        }
    }
}

void MotorBroker::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    struct can_frame frame;

    frame.can_id = CAN::can_ids::CMD_VEL;
    frame.can_dlc = sizeof(CAN::CmdVel);

    float linear_x_µm_s_float = msg->linear.x * 10e6;
    int32_t linear_x_µm_s = linear_x_µm_s_float;
    float angular_z_µrad_s_float = msg->angular.z * 10e6;
    int32_t angular_z_µrad_s = angular_z_µrad_s_float;

    frame.data[0] = (linear_x_µm_s >> 24) & 0xFF;
    frame.data[1] = (linear_x_µm_s >> 16) & 0xFF;
    frame.data[2] = (linear_x_µm_s >> 8) & 0xFF;
    frame.data[3] = (linear_x_µm_s) & 0xFF;
    frame.data[4] = (angular_z_µrad_s >> 24) & 0xFF;
    frame.data[5] = (angular_z_µrad_s >> 16) & 0xFF;
    frame.data[6] = (angular_z_µrad_s >> 8) & 0xFF;
    frame.data[7] = (angular_z_µrad_s) & 0xFF;
    send_can_frame(frame);

    /*frame.can_id = CAN::can_ids::CMD_VEL_FLOAT;
    frame.can_dlc = sizeof(CAN::CmdVelFloat);
    float l_linear_x_float = msg->linear.x;
    float l_angular_z_float = msg->angular.z;
    memcpy(&(frame.data), &(l_linear_x_float), sizeof(float));
    memcpy(&(frame.data) + sizeof(float), &(l_angular_z_float), sizeof(float));
    send_can_frame(frame);*/
}

void MotorBroker::motorsParametersCallback(const krabi_msgs::msg::MotorsParameters::SharedPtr msg)
{
    struct can_frame frame;

    frame.can_id = CAN::can_ids::MOTOR_BOARD_CURRENT_INPUT;
    frame.can_dlc = sizeof(CAN::MotorBoardCurrentInput);

    uint16_t l_max_current_left = msg->max_current_left;
    uint16_t l_max_current_right = msg->max_current_right;
    uint16_t l_max_current = msg->max_current;

    frame.data[0] = l_max_current_left >> 8;
    frame.data[1] = l_max_current_left;
    frame.data[2] = l_max_current_right >> 8;
    frame.data[3] = l_max_current_right;
    frame.data[4] = l_max_current >> 8;
    frame.data[5] = l_max_current;
    send_can_frame(frame);
}

void MotorBroker::motorsCmdCallback(const krabi_msgs::msg::MotorsCmd::SharedPtr msg)
{
    struct can_frame frame;

    frame.can_id = CAN::can_ids::MOTOR_BOARD_CMD_INPUT;
    frame.can_dlc = sizeof(CAN::MotorBoardCmdInput);

    frame.data[0] = msg->enable_motors;
    frame.data[1] = msg->override_pwm;
    frame.data[2] = msg->pwm_override_left >> 8;
    frame.data[3] = msg->pwm_override_left % 256;
    frame.data[4] = msg->pwm_override_right >> 8;
    frame.data[5] = msg->pwm_override_right % 256;
    frame.data[6] = msg->reset_encoders;
    send_can_frame(frame);
}

void MotorBroker::motorsEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    struct can_frame frame;

    frame.can_id = CAN::can_ids::MOTOR_BOARD_ENABLE;
    frame.can_dlc = sizeof(CAN::MotorEnable);

    frame.data[0] = msg->data;
    send_can_frame(frame);
}

void MotorBroker::publish_analog_sensors(const int16_t& battery_voltage_mV)
{
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = sensor_msgs::msg::BatteryState();
    msg.voltage = battery_voltage_mV / 1000.0; // Convert mV to V
    msg.present = true;

    RCLCPP_INFO(this->get_logger(), "Publishing AnalogSensors: battery_mV=%d", battery_voltage_mV);

    battery_pub_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
