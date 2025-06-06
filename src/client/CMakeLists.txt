cmake_minimum_required(VERSION 3.14)

project(Robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(${PROJECT_NAME}
    robot.cpp
)

include(FetchContent)

set(WITH_DOCS OFF CACHE BOOL "Mosquitto: Disable documentation build" FORCE)
set(WITH_APPS OFF CACHE BOOL "Mosquitto: Disable application build" FORCE)
set(WITH_TLS OFF CACHE BOOL "Mosquitto: Disable SSL/TLS support" FORCE)
set(WITH_PIC ON CACHE BOOL "Mosquitto: Build with Position Independent Code" FORCE)
set(WITH_CPP ON CACHE BOOL "Mosquitto: Build C++ wrapper (libmosquittopp)" FORCE)
set(WITH_STATIC_LIBRARIES ON CACHE BOOL "Mosquitto: Build static libraries" FORCE)
set(WITH_SHARED_LIBRARIES OFF CACHE BOOL "Mosquitto: Do NOT build shared libraries" FORCE)

FetchContent_Declare(
    mosquitto
    GIT_REPOSITORY https://github.com/eclipse/mosquitto.git
    GIT_TAG        a196c2b244f248072a6b3ac8fb3f00ce0ff63dea
)
FetchContent_MakeAvailable(mosquitto)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${mosquitto_SOURCE_DIR}/include
    ${mosquitto_SOURCE_DIR}/lib/cpp
    ${mosquitto_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    mosquittopp
    mosquitto
    pthread
)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)