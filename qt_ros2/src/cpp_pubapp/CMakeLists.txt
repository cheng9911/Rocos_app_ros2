cmake_minimum_required(VERSION 3.10)
project(cpp_pubapp VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)


# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


set(PROJECT_SOURCES
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.h
        src/mainwindow.ui
        
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(cpp_pubapp
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET QT_demo APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(cpp_pubapp SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(cpp_pubapp
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(cpp_pubapp  Qt${QT_VERSION_MAJOR}::Widgets)
ament_target_dependencies(cpp_pubapp rclcpp std_msgs sensor_msgs)

set_target_properties(cpp_pubapp PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS cpp_pubapp
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)




# Default to C++14





# add_executable(joint_state_publisher src/publisher_joint_state.cpp)
#target_link_libraries(talker  ${catkin_LIBRARIES} rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )
#target_link_libraries(joint_state_publisher  ${catkin_LIBRARIES} rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)


install(TARGETS cpp_pubapp
DESTINATION lib/${PROJECT_NAME})


# if(QT_VERSION_MAJOR EQUAL 6)
#   qt_finalize_executable(cpp_pubapp)
# endif()



ament_package()
