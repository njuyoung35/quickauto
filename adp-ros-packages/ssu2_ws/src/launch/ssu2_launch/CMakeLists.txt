cmake_minimum_required(VERSION 3.8)
project(ssu2_launch)

# PRESERVED_JINJA2_CODEGEN_START

# PRESERVED_JINJA2_CODEGEN_END

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# PRESERVED_JINJA2_CODEGEN_START

# PRESERVED_JINJA2_CODEGEN_END

ament_auto_package(
  INSTALL_TO_SHARE
  rviz
  config
  launch
)