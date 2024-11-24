cmake_minimum_required(VERSION 3.5)
project(ArtificialHorizon)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(artificial_horizon main.cpp horizonwidget.cpp)

target_link_libraries(artificial_horizon Qt5::Widgets)

