cmake_minimum_required(VERSION 3.5)

project(pid)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)

include_directories(include 
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(controller src/pid/controller.cpp src/pid/pid.cpp)
target_link_libraries(controller
                      ${rclcpp_LIBRARIES}
                      ${rmw_implementation_LIBRARIES}
                      ${std_msgs_LIBRARIES})
install(TARGETS controller DESTINATION lib/${PROJECT_NAME})


function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "std_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(pid servo_sim)
#custom_executable(pid controller)
custom_executable(pid setpoint)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
