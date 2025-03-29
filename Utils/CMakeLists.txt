cmake_minimum_required(VERSION 3.10)
project(CoffeeRobotSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(CoffeeRobotSystem
        src/Application.cpp
        src/Application.h
        src/Camera.cpp
        src/Camera.h
        src/Drink.cpp
        src/Drink.h
        src/iRobot.h
        src/Order.cpp
        src/Order.h
        src/Robot.cpp
        src/Robot.h
        src/Server.cpp
        src/Server.h
        src/Student.cpp
        src/Student.h
        main.cpp
)

target_include_directories(CoffeeRobotSystem PRIVATE src)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(CoffeeRobotSystem PRIVATE -g)
endif()