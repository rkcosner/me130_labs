cmake_minimum_required(VERSION 3.16)

project(balance_encoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

find_library(
    GPIOD_LIBRARY
    gpiod
    REQUIRED
)

add_executable(
    balance_encoder_pwmpwm
    balance_encoder_pwmpwm.cpp
)

target_link_libraries(
    balance_encoder_pwmpwm
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)

add_executable(
    balance_encoder_pwmdir
    balance_encoder_pwmdir.cpp
)

target_link_libraries(
    balance_encoder_pwmdir
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)