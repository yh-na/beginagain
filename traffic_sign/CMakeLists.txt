cmake_minimum_required(VERSION 2.8.3)
project(traffic_sign)


add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

file(GLOB SRCS src/*.cpp)
set(OutDir ${PROJECT_SOURCE_DIR}/../../bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
sensor_msgs
cv_bridge
image_transport
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES include
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(traffic_sign_lib src/Gradient.cpp)
add_executable(traffic_sign src/main.cpp)
target_link_libraries(traffic_sign  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

