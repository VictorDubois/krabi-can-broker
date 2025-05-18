#include "actuators_broker.hpp"

CanActuatorBroker::CanActuatorBroker()
  : GenericCanBroker()
{
    // Create subscribers for each message type
    actuators2025_sub_ = this->create_subscription<krabi_msgs::msg::Actuators2025>(
      "actuators2025",
      10,
      std::bind(&CanActuatorBroker::servoCallback, this, std::placeholders::_1));
    remaining_time_sub_ = this->create_subscription<builtin_interfaces::msg::Duration>(
      "/remaining_time",
      10,
      std::bind(&CanActuatorBroker::remainingTimeCallback, this, std::placeholders::_1));

    battery_power_pub_
      = this->create_publisher<sensor_msgs::msg::BatteryState>("power_battery", 10);
    battery_elec_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("elec_battery", 10);
    vacuum_pub_ = this->create_publisher<std_msgs::msg::Float32>("vacuum", 10);
    stepper_info_pub_ = this->create_publisher<krabi_msgs::msg::InfosStepper>("stepper_info", 10);
    AX12_pub_1 = this->create_publisher<krabi_msgs::msg::AX12Info>("ax12_1_info", 10);
    AX12_pub_2 = this->create_publisher<krabi_msgs::msg::AX12Info>("ax12_2_info", 10);
    AX12_pub_3 = this->create_publisher<krabi_msgs::msg::AX12Info>("ax12_3_info", 10);
    AX12_pub_4 = this->create_publisher<krabi_msgs::msg::AX12Info>("ax12_4_info", 10);
    digitalReads_pub = this->create_publisher<std_msgs::msg::Byte>("digitalRead", 10);

    m_is_blue = this->get_parameter("isBlue").as_bool();

    // Start a separate thread to listen to CAN messages
    can_receiver_thread_ = std::thread(&CanActuatorBroker::receive_can_messages, this);

    /*cmd_vel_sub_ = this->create_subscription<CmdVel>(
        "cmd_vel", 10, std::bind(&CanActuatorBroker::cmdVelCallback, this,
       std::placeholders::_1));*/

    // Additional subscribers for other message types...
}

void CanActuatorBroker::receive_can_messages()
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

        if (frame.can_id == CAN::can_ids::STEPPER_INFO && frame.can_dlc == sizeof(CAN::StepperInfo))
        {
            CAN::StepperInfo stepper_info;
            stepper_info.distance_to_go = frame.data[1] | (frame.data[0] << 8);
            stepper_info.homing_sequences_done = frame.data[2];
            stepper_info.homing_switches_on = frame.data[3];
            publish_stepper_info(&stepper_info);
        }

        if (frame.can_id == CAN::can_ids::ANALOG_SENSORS
            && frame.can_dlc == sizeof(CAN::AnalogSensors))
        {
            CAN::AnalogSensors analogSensors;
            analogSensors.battery_power_mV = frame.data[1] | (frame.data[0] << 8);
            analogSensors.battery_elec_mV = frame.data[3] | (frame.data[2] << 8);
            analogSensors.vacuum_1 = frame.data[5] | (frame.data[4] << 8);
            analogSensors.vacuum_2 = frame.data[7] | (frame.data[6] << 8);
            publish_analog_sensors(analogSensors.battery_power_mV, analogSensors.battery_elec_mV);
            publish_vacuum(analogSensors.vacuum_1);
        }

        if (frame.can_id == CAN::can_ids::DIGITAL_INPUTS
            && frame.can_dlc == sizeof(CAN::DigitalInputs))
        {
            publish_digital_sensors(frame.data[0]);
        }

        if (frame.can_id >= CAN::can_ids::AX12_R1 && frame.can_id <= CAN::can_ids::AX12_R6
            && frame.can_dlc == sizeof(CAN::AX12Read))
        {
            CAN::AX12Read ax12_read;
            ax12_read.hardwareErrorStatus = frame.data[0];
            ax12_read.current_position = frame.data[2] | frame.data[1] << 8;
            ax12_read.presentTemperature = frame.data[3];
            ax12_read.presentCurrent = frame.data[5] | frame.data[4] << 8;
            ax12_read.moving = frame.data[6];
            ax12_read.mode = frame.data[7];

            publish_AX12(ax12_read, frame.can_id);
        }
    }
}
void CanActuatorBroker::remainingTimeCallback(
  const builtin_interfaces::msg::Duration::SharedPtr msg)
{
    m_remaining_time = msg->sec;
}

// Servo callback
void CanActuatorBroker::servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg)
{
    struct can_frame frame;
    frame.can_id = CAN::can_ids::SERVO_1;
    frame.can_dlc = sizeof(CAN::ServoMessage);
    frame.data[0] = msg->servo_1.angle;
    frame.data[1] = msg->servo_1.speed;
    frame.data[2] = msg->servo_2.angle;
    frame.data[3] = msg->servo_2.speed;
    frame.data[4] = msg->servo_3.angle;
    frame.data[5] = msg->servo_3.speed;
    frame.data[6] = msg->servo_4.angle;
    frame.data[7] = msg->servo_4.speed;
    send_can_frame(frame);

    frame.can_id = CAN::can_ids::SERVO_2;
    frame.can_dlc = sizeof(CAN::ServoMessage);
    frame.data[0] = msg->servo_5.angle;
    frame.data[1] = msg->servo_5.speed;
    frame.data[2] = msg->servo_6.angle;
    frame.data[3] = msg->servo_6.speed;
    frame.data[4] = msg->servo_7.angle;
    frame.data[5] = msg->servo_7.speed;
    frame.data[6] = msg->servo_8.angle;
    frame.data[7] = msg->servo_8.speed;
    send_can_frame(frame);

    frame.can_id = CAN::can_ids::STEPPER_CMD;
    frame.can_dlc = sizeof(CAN::Stepper);
    frame.data[0] = msg->stepper_1.speed >> 8;
    frame.data[1] = msg->stepper_1.speed % 256;
    frame.data[2] = msg->stepper_1.accel >> 8;
    frame.data[3] = msg->stepper_1.accel % 256;
    frame.data[4] = msg->stepper_1.position >> 8;
    frame.data[5] = msg->stepper_1.position % 256;
    frame.data[6] = msg->stepper_1.current;
    frame.data[7] = msg->stepper_1.mode;
    send_can_frame(frame);

    frame.can_id = CAN::can_ids::SCORE;
    frame.can_dlc = sizeof(CAN::Score);
    for (int i = 0; i < 8; i++)
    {
        frame.data[i] = 0;
    }
    frame.data[0] = msg->score;
    frame.data[1] = m_remaining_time;
    frame.data[2] = m_is_blue;
    send_can_frame(frame);

    sendAX12Write(msg->ax12_1, CAN::can_ids::AX12_W1);
    sendAX12Write(msg->ax12_2, CAN::can_ids::AX12_W2);
    sendAX12Write(msg->ax12_3, CAN::can_ids::AX12_W3);
    sendAX12Write(msg->ax12_4, CAN::can_ids::AX12_W4);
}

void CanActuatorBroker::sendAX12Write(const krabi_msgs::msg::AX12Cmd ax12_msg, CAN::can_ids id)
{
    struct can_frame frame;

    frame.can_id = id;
    frame.can_dlc = sizeof(CAN::AX12Write);
    frame.data[0] = ax12_msg.mode;
    frame.data[1] = (ax12_msg.position >> 8) & 0xFF;
    frame.data[2] = ax12_msg.position & 0xFF;
    frame.data[3] = ax12_msg.max_accel;
    frame.data[4] = ax12_msg.max_speed;
    frame.data[5] = ax12_msg.torque_enable;
    frame.data[6] = ax12_msg.temperature_limit;
    frame.data[7] = ax12_msg.current_limit;
    send_can_frame(frame);
}

void CanActuatorBroker::publish_digital_sensors(const uint8_t& a_digital_intputs)
{
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = std_msgs::msg::Byte();
    msg.data = a_digital_intputs;

    RCLCPP_INFO(this->get_logger(),
                "Publishing a_digital_intputs: %d %d %d %d %d %d %d %d",
                a_digital_intputs << 0 & 0x1,
                a_digital_intputs << 1 & 0x1,
                a_digital_intputs << 2 & 0x1,
                a_digital_intputs << 3 & 0x1,
                a_digital_intputs << 4 & 0x1,
                a_digital_intputs << 5 & 0x1,
                a_digital_intputs << 6 & 0x1,
                a_digital_intputs << 7 & 0x1);

    digitalReads_pub->publish(msg);
}

void CanActuatorBroker::publish_analog_sensors(const int16_t& battery_power_mV,
                                               const int16_t& battery_elec_mV)
{
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = sensor_msgs::msg::BatteryState();
    msg.voltage = battery_power_mV / 1000.0; // Convert mV to V
    msg.present = true;

    RCLCPP_INFO(this->get_logger(), "Publishing Battery Power: battery_mV=%d", battery_power_mV);

    battery_power_pub_->publish(msg);

    msg.voltage = battery_elec_mV / 1000.0; // Convert mV to V
    msg.present = true;

    RCLCPP_INFO(this->get_logger(), "Publishing Battery Elec: battery_mV=%d", battery_elec_mV);

    battery_elec_pub_->publish(msg);
}

void CanActuatorBroker::publish_vacuum(const int16_t& vacuum)
{
    auto msg = std_msgs::msg::Float32();
    msg.data = vacuum / 1000.0;

    RCLCPP_INFO(this->get_logger(), "Publishing Vacuum: %d", vacuum);

    vacuum_pub_->publish(msg);
}

void CanActuatorBroker::publish_stepper_info(const CAN::StepperInfo* stepper_info)
{
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = krabi_msgs::msg::InfosStepper();
    msg.distance_to_go = stepper_info->distance_to_go;
    msg.homing_sequences_done = stepper_info->homing_sequences_done;
    msg.homing_switch_on = stepper_info->homing_switches_on;

    RCLCPP_INFO(this->get_logger(),
                "Publishing StepperInfo: distance_to_go=%d, homing_sequences_done = %d",
                stepper_info->distance_to_go,
                stepper_info->homing_sequences_done);

    stepper_info_pub_->publish(msg);
}

void CanActuatorBroker::publish_AX12(const CAN::AX12Read& ax12_read, int id)
{
    auto ax12_read_msg = krabi_msgs::msg::AX12Info();

    ax12_read_msg.hardware_error_status = ax12_read.hardwareErrorStatus;
    ax12_read_msg.current_position = ax12_read.current_position;
    ax12_read_msg.present_temperature = ax12_read.presentTemperature;
    ax12_read_msg.present_current = ax12_read.presentCurrent;
    ax12_read_msg.moving = ax12_read.moving;
    ax12_read_msg.mode = ax12_read.mode;
    if (id == CAN::AX12_R1)
    {
        AX12_pub_1->publish(ax12_read_msg);
    }
    else if (id == CAN::AX12_R2)
    {
        AX12_pub_2->publish(ax12_read_msg);
    }
    else if (id == CAN::AX12_R3)
    {
        AX12_pub_3->publish(ax12_read_msg);
    }
    else if (id == CAN::AX12_R4)
    {
        AX12_pub_4->publish(ax12_read_msg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanActuatorBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
