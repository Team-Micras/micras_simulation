cmake_minimum_required(VERSION 3.16.3)
project(micras_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(ROS_CODE_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

foreach(dependency ${ROS_CODE_DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include/micras_simulation
  cfg
  MicrasFirmware/inc
)

add_library(${PROJECT_NAME}_lib
  cfg/target.cpp
  src/lib/button.cpp
  src/lib/distance_sensors.cpp
  src/lib/imu.cpp
  src/lib/led.cpp
  src/lib/locomotion.cpp
  src/lib/odometry.cpp
  src/lib/rotary_sensor.cpp
  src/lib/torque_sensor.cpp
  MicrasFirmware/src/controller/micras_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib ${ROS_CODE_DEPS})

add_executable(micras_node
  src/bin/micras_node.cpp
)

ament_target_dependencies(micras_node ${ROS_CODE_DEPS})
target_link_libraries(micras_node ${PROJECT_NAME}_lib)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY gazebo
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS micras_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_CODE_DEPS})
ament_package()
