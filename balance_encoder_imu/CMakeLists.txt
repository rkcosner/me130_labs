cmake_minimum_required(VERSION 3.16)

project(balance_encoder_imu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

find_library(
    GPIOD_LIBRARY
    gpiod
    REQUIRED
)

add_executable(
    balance_encoder_imu
    balance_encoder_imu.cpp
)

target_link_libraries(
    balance_encoder_imu
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)