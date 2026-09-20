cmake_minimum_required(VERSION 3.16)
project(me130_pendulum)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(me130_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_library(GPIOD_LIBRARY gpiod REQUIRED)

# Shared hardware layer: sysfs PWM, motor GPIO, quadrature encoder, MPU6050.
add_library(me130_hardware src/hardware.cpp)
target_include_directories(me130_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(me130_hardware PUBLIC ${GPIOD_LIBRARY} Threads::Threads)

# node name -> source
set(ME130_NODES
  imu_node
  encoder_node
  motor_node
  controller_node
  step_sequence_node
  sine_sweep_node
  motor_characterization_node
  logger_node
  keyboard_node)

foreach(node ${ME130_NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} me130_hardware)
  ament_target_dependencies(${node} rclcpp std_srvs me130_interfaces)
endforeach()

install(TARGETS me130_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${ME130_NODES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Analysis scripts, reachable as: ros2 run me130_pendulum step_response.py
install(PROGRAMS
  scripts/step_response.py
  scripts/freq_response.py
  scripts/plot_characterization.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
