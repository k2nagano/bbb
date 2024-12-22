cmake_minimum_required(VERSION 3.5)

project(UdpJpegReceiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(UdpJpegReceiver
    main.cpp
    mainwindow.cpp
    mainwindow.h
    udpthread.cpp
    udpthread.h
)

target_link_libraries(UdpJpegReceiver Qt5::Widgets)

