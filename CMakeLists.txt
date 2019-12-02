cmake_minimum_required(VERSION 2.8.12)
project(CenterLine)
## Compile as C++11
add_compile_options(-std=c++11)
## Specify additional locations of header files
include_directories(
  include
)
## Add cmake target dependencies of the library
add_executable(lineExt src/main.cpp src/FileAccess.cpp src/Calculate.cpp src/Optimizer.cpp)
## target_link_libraries(CenterLine)
