cmake_minimum_required(VERSION 3.5)

project(UdpH264Streamer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

add_executable(UdpH264Streamer
    main.cpp
    mainwindow.cpp
    mainwindow.h
    gstthread.cpp
    gstthread.h
)

target_include_directories(UdpH264Streamer PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(UdpH264Streamer PRIVATE Qt5::Widgets ${GST_LIBRARIES})

