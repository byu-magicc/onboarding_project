cmake_minimum_required(VERSION 3.8)
project(onboarding_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(rviz_hallway_publisher
  src/rviz_hallway_publisher.cpp)
target_include_directories(rviz_hallway_publisher
   PRIVATE
    include/rviz_hallway_publisher
    ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(rviz_hallway_publisher rclcpp visualization_msgs Eigen3)
target_link_libraries(rviz_hallway_publisher ${ament_LIBRARIES})
install(
  TARGETS rviz_hallway_publisher
  DESTINATION lib/${PROJECT_NAME})



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

ament_package()
