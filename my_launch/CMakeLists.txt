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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install param files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY
  map
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(init_pos_pub src/init_pos_pub.cpp)
add_executable(create_path src/create_path.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(init_pos_pub
  rclcpp
  geometry_msgs
)

ament_target_dependencies(create_path
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install the executable
install(TARGETS
init_pos_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
create_path
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
