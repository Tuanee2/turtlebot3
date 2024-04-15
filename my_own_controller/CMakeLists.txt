cmake_minimum_required(VERSION 3.8)
project(my_own_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# Add your executable
add_executable(my_own_controller src/test.cpp)
add_executable(draw_circle src/draw_circle.cpp)
add_executable(draw_square src/draw_square.cpp)

# Link dependencies
target_link_libraries(my_own_controller
  ${rclcpp_LIBRARIES}
)
target_link_libraries(draw_circle
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
target_link_libraries(draw_square
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

# Install
install(TARGETS
  my_own_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  draw_circle
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  draw_square
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_target_dependencies(my_own_controller
  rclcpp
)
ament_target_dependencies(draw_circle
  rclcpp
  geometry_msgs
)
ament_target_dependencies(draw_square
  rclcpp
  geometry_msgs
)

ament_package()
