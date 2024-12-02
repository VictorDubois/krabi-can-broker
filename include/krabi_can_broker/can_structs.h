enum can_ids {
  // priority, in case we need to shutdown
  DIGITAL_OUTPUTS=1, // DigitalOutputs

  // 10/20Hz
  SERVO_1=10, // ServoMessage
  SERVO_2, // ServoMessage
  SERVO_3, // ServoMessage
  SERVO_4, // ServoMessage
  SERVO_5, // ServoMessage
  SERVO_6, // ServoMessage
  SERVO_7, // ServoMessage
  
  // 10/20Hz
  CMD_VEL, // CmdVel
  
  // High frequency, ~100Hz
  ODOMETRY_XY, // OdometryXY
  ODOMETRY_THETA, // OdometryTheta
  ODOMETRY_SPEED, // SpeedOdometry
  
  // Low frequency
  MOTOR_BOARD_CURRENT_OUPUT, // MotorBoardCurrentOutput
  MOTOR_BOARD_CMD_INPUT, // MotorBoardCmdInput
  MOTOR_BOARD_CURRENT_INPUT, // MotorBoardCurrentInput

  ANALOG_SENSORS = 60, // AnalogSensors
  
  SCORE=100 // Score
};


#pragma pack(push, 1)  // Ensure tight packing of the struct
struct ServoMessage {
    uint8_t angle_s1;
    uint8_t speed_s1; // 0 means disable the servo
    uint8_t angle_s2;
    uint8_t speed_s2; // 0 means disable the servo
    uint8_t angle_s3;
    uint8_t speed_s3; // 0 means disable the servo
    uint8_t angle_s4;
    uint8_t speed_s4; // 0 means disable the servo
};

struct Score {
    uint32_t score;  // 4 bytes
};

struct DigitalOutputs {
    // array of ouputs
    int16_t enable_outputs; // 2 bytes
    int8_t enable_power; // 1 byte
};

struct AnalogSensors {
    int16_t battery_mV; // 2 bytes    
    int16_t vacuum_1; // 2 bytes
    int16_t vacuum_2; // 2 bytes
    int16_t vacuum_3; // 2 bytes
};

struct CmdVel {
    float linear_x_m ; // 4 bytes
    float angular_z_rad ; // 4 bytes
};

struct MotorBoardCurrentInput {
    int16_t max_current_left_mA; // 2 bytes
    int16_t max_current_right_mA; // 2 bytes
    int16_t max_current_mA; // 2 bytes
};

struct MotorBoardCmdInput {
    int8_t enable_motors; // 1 bytes
    int8_t override_PWM; // 1 bytes
    int16_t PWM_override_left; // 2 bytes - valeur brute à envoyer au PWM
    int16_t PWM_override_right; // 2 bytes - valeur brute à envoyer au PWM
    int8_t reset_encoders; // 1 bytes
};


struct MotorBoardCurrentOutput {
    int16_t current_left_mA; // 2 bytes
    int16_t current_right_mA; // 2 bytes
    int16_t remaining_timeout_ms; // 2 bytes
    int16_t motor_battery_mV; // 2 bytes
};

struct OdometryXY {
    float poseX_m ; // 4 bytes
    float poseY_m ; // 4 bytes
};

struct OdometryTheta {
    float angleRz ; // 4 bytes
};

struct SpeedOdometry {
    float speedVx_m_s ; // 4 bytes
    float speedWz_rad_s ; // 4 bytes
};

struct Stepper {
    uint16_t speed; // mm/s 0 => disable
    uint16_t accel; // mm/s²
    int16_t position; // mm
    uint8_t current; // *50mA
    uint8_t mode; // 0 => disable, 10 => position, 20 => speed, 30 => homing
};

struct StepperInfo {
    uint16_t distance_to_go; // mm
};

#pragma pack(pop)

/*
class CANMessage {
  public : uint32_t id = 0 ;  // Frame identifier
  public : bool ext = false ; // false -> standard frame, true -> extended frame
  public : bool rtr = false ; // false -> data frame, true -> remote frame
  public : uint8_t idx = 0 ;  // This field is used by the driver
  public : uint8_t len = 0 ;  // Length of data (0 ... 8)
  public : union {
    uint64_t data64        ; // Caution: subject to endianness
    int64_t  data_s64      ; // Caution: subject to endianness
    uint32_t data32    [2] ; // Caution: subject to endianness
    int32_t  data_s32  [2] ; // Caution: subject to endianness
    float    dataFloat [2] ; // Caution: subject to endianness
    uint16_t data16    [4] ; // Caution: subject to endianness
    int16_t  data_s16  [4] ; // Caution: subject to endianness
    int8_t   data_s8   [8] ;
    uint8_t  data      [8] = {0, 0, 0, 0, 0, 0, 0, 0} ;
  } ;
} ;
*/

