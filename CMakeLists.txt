cmake_minimum_required(VERSION 2.8.12)

project(onlineControl)

find_package(DART 6.2.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

#add_compile_options(-std=c++11)
#add_compile_options(-std=c++11 -DEIGEN_STACK_ALLOCATION_LIMIT=0)
add_compile_options(-std=c++11 -DEIGEN_STACK_ALLOCATION_LIMIT=0 -Ofast)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt)
