cmake_minimum_required(VERSION 3.10)
project(test_actuators)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package(TBB REQUIRED)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(px4_msgs QUIET)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

set(PROTO_FILE ../../msgs/CommandMotorSpeed.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})

add_executable(test_actuators
  test_actuators.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_link_libraries(test_actuators
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  TBB::tbb
)

if (ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND)
  add_executable(ros_gazebo_bridge
    ros_gazebo_bridge.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
  )

  target_link_libraries(ros_gazebo_bridge
    ${GAZEBO_LIBRARIES}
    ${Protobuf_LIBRARIES}
    TBB::tbb
  )

  ament_target_dependencies(ros_gazebo_bridge
    rclcpp
    std_msgs
  )

  if (px4_msgs_FOUND)
    target_compile_definitions(ros_gazebo_bridge PRIVATE HAVE_PX4_MSGS=1)
    ament_target_dependencies(ros_gazebo_bridge px4_msgs)
  else()
    message(STATUS "Building ros_gazebo_bridge without px4_msgs support")
  endif()
else()
  message(STATUS "Skipping ros_gazebo_bridge: ament_cmake/rclcpp/std_msgs not found")
endif()
