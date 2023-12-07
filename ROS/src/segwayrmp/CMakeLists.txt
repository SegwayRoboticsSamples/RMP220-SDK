cmake_minimum_required(VERSION 2.8.3)
project(segwayrmp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -funroll-loops -fpic -Wall -Wno-sign-compare -Werror=return-type -g")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fpic")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  segway_msgs
)

message("-- Segway Robtics Notice: This external lib and include path can be changed by users.")

#This setting can be changed by users.
set(EXTERNAL_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/../../LibAPI/include)
message("-- Segway Robtics Notice: The external inlude path is " ${EXTERNAL_INCLUDE_PATH})

#This setting can be changed by users.
set(EXTERNAL_LIB_PATH ${CMAKE_SOURCE_DIR}/../../LibAPI/lib)
message("-- Segway Robtics Notice: The external lib path is " ${EXTERNAL_LIB_PATH})


if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message("-- Segway Robtics Notice: This is aarch64 cpu")
    set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_arm64-v8a.so)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
	message("-- Segway Robtics Notice: This is x86_64 cpu")
  set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_x86_64.so)
else()
  message("-- Segway Robotics Notice: The lib_xxx.so is only supported to x86_64 and aarch64,sorry!")
  return()
endif()
message( "-- Segway Robtics Notice: EXTERNAL_LIB name is " ${EXTERNAL_LIB} )


include_directories(
  include
  ${EXTERNAL_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ChassisResponseTest
  tools/ChassisResponseTest.cpp
)
add_dependencies(ChassisResponseTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ChassisResponseTest
  ${catkin_LIBRARIES}
)


add_executable(SmartCar
  src/SmartCar.cpp
  src/robot.cpp
  src/Ge_encoder_odometry.cpp
)
add_dependencies(SmartCar
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(SmartCar
  ${catkin_LIBRARIES} 
  ${EXTERNAL_LIB}
)

