cmake_minimum_required(VERSION 3.22)
project(ldlidar_driver)

# Set include directories for header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find Boost (dynamic linking by default)
find_package(Boost REQUIRED COMPONENTS system thread)

# Define source files for Linux
set(LDLIDAR_DRIVER_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver_linux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_dataprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_protocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/log_module.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/network_socket_interface_linux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/serial_interface_linux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/slbf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tofbf.cpp
)

# Create static library
add_library(ldlidar_driver STATIC ${LDLIDAR_DRIVER_SOURCE})
target_link_libraries(ldlidar_driver PRIVATE Boost::system Boost::thread pthread)

# Create executable
add_executable(lidar_quadrants lidar_terminal_output.cpp)
target_link_libraries(lidar_quadrants PRIVATE ldlidar_driver Boost::system Boost::thread pthread)