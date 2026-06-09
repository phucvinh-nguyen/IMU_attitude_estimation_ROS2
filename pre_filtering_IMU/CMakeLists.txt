cmake_minimum_required(VERSION 3.8)
project(pre_filtering_imu)

# ROS 2 mặc định sử dụng C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_library(imu_filter
  src/median_filter.cpp
  src/butterworth_filter.cpp
)

target_include_directories(imu_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_filter rclcpp sensor_msgs std_msgs)


add_executable(imu_prefilter_node 
  src/main.cpp 
  src/imu_prefilter_node.cpp
)
target_link_libraries(imu_prefilter_node imu_filter)
ament_target_dependencies(imu_prefilter_node rclcpp sensor_msgs std_msgs)


install(TARGETS imu_filter
  EXPORT export_pre_filtering_imu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS imu_prefilter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_pre_filtering_imu HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs std_msgs)

ament_package()
