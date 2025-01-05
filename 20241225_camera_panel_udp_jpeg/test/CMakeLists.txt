cmake_minimum_required(VERSION 3.5)

project(UdpJpegReceiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(UdpJpegReceiver
    test.cc
    CameraPanel.cc
    # CameraPanel.hh
)

target_link_libraries(UdpJpegReceiver Qt5::Widgets)

