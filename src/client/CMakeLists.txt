cmake_minimum_required(VERSION 3.10)
project(RobotClient LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_library(MOSQUITTO_LIB mosquitto REQUIRED)

include_directories(include)

add_executable(robot_client
    src/main.cpp
    src/pin_manager.cpp
    src/motion_controller.cpp
    src/network_handler.cpp
    src/command_processor.cpp
)

target_link_libraries(robot_client
    PRIVATE
    Threads::Threads
    ${MOSQUITTO_LIB}
)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    target_link_libraries(robot_client PRIVATE -lrt)
endif()
