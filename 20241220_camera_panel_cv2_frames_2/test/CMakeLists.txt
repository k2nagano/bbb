cmake_minimum_required(VERSION 3.5)
project(UdpH264OpenCVRecorder)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(UdpH264OpenCVRecorder main.cpp frame_receiver.cpp frame_receiver.h recorder.cpp recorder.h)

target_link_libraries(UdpH264OpenCVRecorder ${OpenCV_LIBS} -lpthread)

