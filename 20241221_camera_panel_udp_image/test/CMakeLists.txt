cmake_minimum_required(VERSION 3.5)

project(UdpImageReceiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets Network REQUIRED)

add_executable(UdpImageReceiver
    main.cpp
    mainwindow.cpp
    mainwindow.h
    udpthread.cpp
    udpthread.h
)

target_link_libraries(UdpImageReceiver Qt5::Widgets Qt5::Network)

