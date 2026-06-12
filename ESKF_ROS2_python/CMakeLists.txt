cmake_minimum_required(VERSION 3.8)
project(eskf_python)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


install(PROGRAMS
  scripts/eskf_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_package()
