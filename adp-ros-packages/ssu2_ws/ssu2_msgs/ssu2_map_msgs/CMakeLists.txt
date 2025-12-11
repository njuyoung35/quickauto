cmake_minimum_required(VERSION 3.8)
project(ssu2_map_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
    "msg/GPSCoordinates.msg"
    "msg/Route.msg"
    "srv/FindShortestRoute.srv"
)

set(msg_dependencies
    std_msgs
)

# PRESERVED_JINJA2_CODEGEN_START

# PRESERVED_JINJA2_CODEGEN_END

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES ${msg_dependencies}
    ADD_LINTER_TESTS
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()