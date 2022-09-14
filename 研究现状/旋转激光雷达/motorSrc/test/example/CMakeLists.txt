cmake_minimum_required(VERSION 3.15)
project(example)

set(CMAKE_PREFIX_PATH "../install")
find_package(motor REQUIRED CONFIG)

add_executable(main main.cpp)
target_compile_options(main PRIVATE -g -std=c++17)

target_link_libraries(main 
    PRIVATE
        motor::motor
)
