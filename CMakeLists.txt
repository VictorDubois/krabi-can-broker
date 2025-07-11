cmake_minimum_required(VERSION 3.8)
project(krabi_can_broker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(krabi_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(goal_strategy REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(actuators_broker src/generic_can_broker.cpp src/actuators_broker.cpp)

ament_target_dependencies(actuators_broker rclcpp krabi_msgs sensor_msgs geometry_msgs std_msgs goal_strategy  tf2_eigen
tf2_ros
tf2_geometry_msgs diagnostic_updater)

target_include_directories(actuators_broker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(actuators_broker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(motor_broker src/generic_can_broker.cpp src/motor_broker.cpp)

ament_target_dependencies(motor_broker rclcpp krabi_msgs sensor_msgs geometry_msgs std_msgs goal_strategy  tf2_eigen
tf2_ros
tf2_geometry_msgs diagnostic_updater)

target_include_directories(motor_broker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(motor_broker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS motor_broker actuators_broker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
