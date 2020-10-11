cmake_minimum_required(VERSION 2.8)
project(C_apf)

find_package(OpenCV REQUIRED)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(apf apf.cpp)

target_link_libraries(
        apf  ${OpenCV_LIBS}
)

