cmake_minimum_required(VERSION 3.8)
project(my_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(OGRE REQUIRED)

# ... your other package dependencies ...
set(CMAKE_AUTOMOC ON)
# Create the plugin's library
set(rviz_plugin_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/src/myCustomDisplay.cpp
)
set(rviz_plugin_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include/myCustomDisplay.hpp
)
add_library(${PROJECT_NAME}_plugin SHARED
  ${rviz_plugin_sources}
  ${rviz_plugin_headers}
)
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  geometry_msgs
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${OGRE_LIBRARIES}
  rviz_common::rviz_common
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install the plugin's library
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install the plugin's header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch, param, map, and rviz files as before

# Install executables
add_executable(init_pos_pub src/init_pos_pub.cpp)
add_executable(create_path src/create_path.cpp)
add_executable(robotController src/robotController.cpp)
add_executable(work_area_monitor src/work_area_monitor.cpp)
ament_target_dependencies(init_pos_pub rclcpp geometry_msgs)
ament_target_dependencies(create_path rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(robotController rclcpp geometry_msgs nav_msgs tf2)
ament_target_dependencies(work_area_monitor rclcpp geometry_msgs)
install(TARGETS
  init_pos_pub
  create_path
  robotController
  work_area_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# If you have any tests, they would go here

# Install other directories (launch, params, map, rviz)
install(DIRECTORY
  launch params map rviz
  DESTINATION share/${PROJECT_NAME}
)

# Finally call ament_package to finish setting up the package
ament_package()
