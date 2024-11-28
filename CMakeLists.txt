cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(simple_planner src/simple_planner.cpp)
target_include_directories(simple_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  simple_planner
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "moveit_core"
)

add_executable(moveit_cpp_tutorial src/moveit_cpp_tutorial.cpp)
target_include_directories(moveit_cpp_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_cpp_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_cpp_tutorial
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "moveit_core"
)

install(TARGETS simple_planner moveit_cpp_tutorial
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
  )

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
