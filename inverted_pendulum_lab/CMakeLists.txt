cmake_minimum_required(VERSION 3.16)

project(pendulum)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

find_library(
    GPIOD_LIBRARY
    gpiod
    REQUIRED
)

add_executable(
    pendulum
    controller.cpp
    pendulum_runner.cpp
)

target_link_libraries(
    pendulum
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)

add_executable(
    pendulum_solution
    controller_solution.cpp
    pendulum_runner.cpp
)

target_link_libraries(
    pendulum_solution
    PRIVATE
    ${GPIOD_LIBRARY}
    Threads::Threads
)
