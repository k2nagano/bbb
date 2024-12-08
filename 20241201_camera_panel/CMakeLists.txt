cmake_minimum_required(VERSION 3.10)

project(sample)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)

add_executable(sample main.cpp)
target_link_libraries(sample Qt5::Widgets ${GST_LIBRARIES})
target_include_directories(sample PRIVATE ${GST_INCLUDE_DIRS})

