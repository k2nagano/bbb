cmake_minimum_required(VERSION 3.5)
project(VideoReceiver)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})

add_executable(VideoReceiver main.cpp)
target_link_libraries(VideoReceiver Qt5::Widgets ${GSTREAMER_LIBRARIES})

