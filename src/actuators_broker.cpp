#include "actuators_broker.hpp"


CanActuatorBroker::CanActuatorBroker() : GenericCanBroker()
{
    // Create subscribers for each message type
    actuators2025_sub_ = this->create_subscription<krabi_msgs::msg::Actuators2025>(
        "actuators2025", 10, std::bind(&CanActuatorBroker::servoCallback, this, std::placeholders::_1));

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("actuators_battery", 10);
        vacuum_pub_ = this->create_publisher<std_msgs::msg::Float32>("vacuum", 10);
        stepper_info_pub_ = this->create_publisher<krabi_msgs::msg::InfosStepper>("stepper_info", 10);


    // Start a separate thread to listen to CAN messages
    can_receiver_thread_ = std::thread(&CanActuatorBroker::receive_can_messages, this);


    /*cmd_vel_sub_ = this->create_subscription<CmdVel>(
        "cmd_vel", 10, std::bind(&CanActuatorBroker::cmdVelCallback, this, std::placeholders::_1));*/

    // Additional subscribers for other message types...
}

void CanActuatorBroker::receive_can_messages() {
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN read error");
            continue;
        }

        if (frame.can_id == CAN::can_ids::STEPPER_INFO && frame.can_dlc == sizeof(CAN::StepperInfo)) {
            CAN::StepperInfo stepper_info;
            stepper_info.distance_to_go = frame.data[1] | (frame.data[0] << 8);
            stepper_info.homing_sequences_done = frame.data[2];
            stepper_info.homing_switches_on = frame.data[3];
            publish_stepper_info(&stepper_info);
        }

        if (frame.can_id == CAN::can_ids::ANALOG_SENSORS && frame.can_dlc == sizeof(CAN::AnalogSensors)) {
            CAN::AnalogSensors analogSensors;
            analogSensors.battery_mV = frame.data[1] | (frame.data[0] << 8);
            analogSensors.vacuum_1 = frame.data[3] | (frame.data[2] << 8);
            analogSensors.vacuum_2 = frame.data[5] | (frame.data[4] << 8);
            publish_analog_sensors(analogSensors.battery_mV);
            publish_vacuum(analogSensors.battery_mV);
        }
    }
}

// Servo callback
void CanActuatorBroker::servoCallback(const krabi_msgs::msg::Actuators2025::SharedPtr msg) {
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
    frame.data[1] = msg->stepper_1.speed%256;
    frame.data[2] = msg->stepper_1.accel >> 8;
    frame.data[3] = msg->stepper_1.accel%256;
    frame.data[4] = msg->stepper_1.position >> 8;
    frame.data[5] = msg->stepper_1.position%256;
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
    send_can_frame(frame);
}

void CanActuatorBroker::publish_analog_sensors(const int16_t &battery_voltage_mV) {
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = sensor_msgs::msg::BatteryState();
    msg.voltage = battery_voltage_mV / 1000.0; // Convert mV to V
    msg.present = true;

    RCLCPP_INFO(this->get_logger(), "Publishing Battery: battery_mV=%d", battery_voltage_mV);

    battery_pub_->publish(msg);
}

void CanActuatorBroker::publish_vacuum(const int16_t &vacuum) {
    auto msg = std_msgs::msg::Float32();
    msg.data = vacuum/1000.0;

    RCLCPP_INFO(this->get_logger(), "Publishing Vacuum: %d", vacuum);

    vacuum_pub_->publish(msg);
}

void CanActuatorBroker::publish_stepper_info(const CAN::StepperInfo* stepper_info) {
    // Convert the AnalogSensors struct to a ROS2 message (e.g., BatteryState for illustration)
    auto msg = krabi_msgs::msg::InfosStepper();
    msg.distance_to_go = stepper_info->distance_to_go;
    msg.homing_sequences_done = stepper_info->homing_sequences_done;
    msg.homing_switch_on = stepper_info->homing_switches_on;

    RCLCPP_INFO(this->get_logger(), "Publishing StepperInfo: distance_to_go=%d, homing_sequences_done = %d", stepper_info->distance_to_go, stepper_info->homing_sequences_done);

    stepper_info_pub_->publish(msg);
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanActuatorBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
