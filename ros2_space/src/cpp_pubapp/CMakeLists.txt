cmake_minimum_required(VERSION 3.10)
project(cpp_pubapp)

list(APPEND CMAKE_PREFIX_PATH /opt/rocos/app)
list(APPEND CMAKE_PREFIX_PATH /opt/grpc)
include_directories(/opt/grpc/include)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rocos_app REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(3rdparty/gflags)

# add_executable(talker src/publisher_member_function.cpp)
add_executable(joint_state_publisher src/publisher_joint_state.cpp)
#target_link_libraries(talker  ${catkin_LIBRARIES} rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )
target_link_libraries(joint_state_publisher  ${catkin_LIBRARIES} rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)
ament_target_dependencies(joint_state_publisher rclcpp std_msgs sensor_msgs)

install(TARGETS
 
joint_state_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME})

file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)
# file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)

ament_package()
