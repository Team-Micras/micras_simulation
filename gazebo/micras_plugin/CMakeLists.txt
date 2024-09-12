cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

project(micras_plugin)

# Find Qt5
find_package(Qt5 COMPONENTS
    Core
    Quick
    QuickControls2
    REQUIRED
)

# Find the Gazebo gui library
find_package(gz-gui8 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")

QT5_ADD_RESOURCES(resources_RCC MicrasPlugin.qrc)

# Generate examples
add_library(MicrasPlugin SHARED ${headers_MOC}
    MicrasPlugin.cpp
    ${resources_RCC}
)

target_include_directories(MicrasPlugin SYSTEM PRIVATE
    ${GZ-COMMON_INCLUDE_DIRS}
    ${GZ-GUI_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Qml_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${Qt5QuickControls2_INCLUDE_DIRS}
)

target_link_directories(MicrasPlugin PRIVATE
    ${GZ-COMMON_LIBRARY_DIRS}
    ${GZ-GUI_LIBRARY_DIRS}
)

target_link_libraries(MicrasPlugin PRIVATE
    ${GZ-GUI_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Qml_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${Qt5QuickControls2_LIBRARIES}
)
