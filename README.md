Message broker, between ROS2 and CAN, for [the robot Krabi](https://github.com/VictorDubois/krabi)

Implements two nodes:
- motor_broker, used to communicate with [Krabi's motors PCB](https://github.com/VictorDubois/Motor-board-STM32-CAN)
- actuators_broker, used to communicate with [Krabi's actuators PCB](https://github.com/VictorDubois/ActuatorBoardCode)

Produces [diagnostics](https://github.com/ros/diagnostics), to help diagnose the robot.

Directly publishes [TF transforms](https://github.com/VictorDubois/krabi-can-broker/blob/main/src/odom_publisher.cpp#L86) from the odometry, to reduce overhead and improve latency: the odom messages are quite high frequency
