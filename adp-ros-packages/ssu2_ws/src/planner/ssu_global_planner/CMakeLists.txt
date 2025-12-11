cmake_minimum_required(VERSION 3.8)
project(ssu_global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(
  include
)

# lanelet2 graph library
add_library(lanelet2_graph
  src/lanelet2_graph.cpp
)
ament_target_dependencies(lanelet2_graph
  rclcpp
)

# main global planner node
add_executable(ssu_global_planner_node
  src/ssu_global_planner_node.cpp
)
ament_target_dependencies(ssu_global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  ament_index_cpp
)
target_link_libraries(lanelet2_graph
  tinyxml2
)

target_link_libraries(ssu_global_planner_node
  lanelet2_graph
  tinyxml2
)

install(TARGETS
  lanelet2_graph
  ssu_global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
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
