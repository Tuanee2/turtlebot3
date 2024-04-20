cmake_minimum_required(VERSION 3.8)
project(test_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_amcl REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install param files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
