cmake_minimum_required(VERSION 3.10)

project(UDP_H264_Receiver_Qt)

find_package(Qt5 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-video-1.0)

add_executable(udp_h264_receiver main.cpp videowidget.cpp)

target_include_directories(udp_h264_receiver PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(udp_h264_receiver PRIVATE Qt5::Widgets Qt5::Multimedia Qt5::MultimediaWidgets ${GSTREAMER_LIBRARIES})

