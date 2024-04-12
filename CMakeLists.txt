# Name: CMakeLists.txt
# Micras Team
# Brief: CMake configuration file for the project
# 04/2023

cmake_minimum_required(VERSION 3.16.3)

project(micras_simulation)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include(cmake/linter.cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(gazebo/micras_plugin)

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

file(GLOB_RECURSE PROJECT_TESTS_BIN CONFIGURE_DEPENDS "MicrasFirmware/tests/bin/*.c*")
file(GLOB_RECURSE PROJECT_TESTS_SOURCES CONFIGURE_DEPENDS "tests/src/*.c*")
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/lib/*.c*")

add_library(${PROJECT_NAME}_lib
    ${PROJECT_SOURCES}
    ${PROJECT_TESTS_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}_lib ${ROS_CODE_DEPS})

add_executable(micras_node
    src/bin/micras_node.cpp
    MicrasFirmware/src/micras_controller.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/config>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MicrasFirmware/include>
)

ament_target_dependencies(micras_node ${ROS_CODE_DEPS})

target_link_libraries(micras_node
    ${PROJECT_NAME}_lib
)

add_dependencies(micras_node MicrasPlugin)

install(DIRECTORY include
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

install(TARGETS MicrasPlugin
    DESTINATION share/${PROJECT_NAME}/micras_plugin
)

foreach(TEST_FILE ${PROJECT_TESTS_BIN})

    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WLE)

    add_executable(${TEST_NAME}
        ${TEST_FILE}
    )

    ament_target_dependencies(${TEST_NAME} ${ROS_CODE_DEPS})

    target_link_libraries(${TEST_NAME}
        ${PROJECT_NAME}_lib
    )

    install(TARGETS ${TEST_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

endforeach()

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
