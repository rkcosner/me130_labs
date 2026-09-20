cmake_minimum_required(VERSION 3.16)

project(motor_characterization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

find_library(
    GPIOD_LIBRARY
    gpiod
    REQUIRED
)

# Current wiring: BD65496MUV EN/IN mode, PWM on INA + DIR on INB (GPIO25).
add_executable(
    motor_characterization
    motor_characterization.cpp
)

target_link_libraries(
    motor_characterization
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)

# Legacy IN/IN wiring (INA=PWM0, INB=PWM1, MODE->GND). Kept for fast-decay
# characterization, which EN/IN mode cannot do. Requires rewiring.
add_executable(
    motor_characterization_pwmpwm
    motor_characterization_pwmpwm.cpp
)

target_link_libraries(
    motor_characterization_pwmpwm
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)
