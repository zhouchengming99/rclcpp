cmake_minimum_required(VERSION 3.5)

project(rclcpp_components)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  # we're building Python C extensions here becuase we define a .srv file
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "srv/LoadNodeComponent.srv"
)

add_library(${PROJECT_NAME} SHARED
  src/rclcpp_components/utilities/get_node_plugins_for_package.cpp
  src/rclcpp_components/utilities/split.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "ament_index_cpp"
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(${PROJECT_NAME} "stdc++fs")
endif()

add_executable(rclcpp_component_container src/rclcpp_component_container.cpp)
ament_target_dependencies(rclcpp_component_container
  "ament_index_cpp"
  "class_loader"
  "pluginlib"
  "rclcpp"
)
target_link_libraries(rclcpp_component_container ${PROJECT_NAME})
rosidl_target_interfaces(rclcpp_component_container
  ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

install(
  TARGETS rclcpp_component_container
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(class_loader)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS rclcpp_components-extras.cmake
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES src/node_main.cpp
  DESTINATION src/rclcpp_components
)
