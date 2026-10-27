cmake_minimum_required(VERSION 3.28)
project(free_space_detection LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# --------------------------------------------------------------------------------
# Find ROS2 dependencies
# --------------------------------------------------------------------------------
find_package(ament_cmake_auto REQUIRED)
#ament_auto_find_build_dependencies(REQUIRED geometry_msgs rosidl_default_generators std_msgs)
ament_auto_find_build_dependencies()

# The executable already uses PROJECT_NAME, so generation needs a distinct target.
rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
    "msg/PerceptionResult.msg"
    LIBRARY_NAME ${PROJECT_NAME}
    DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp)
ament_auto_add_executable(free_space_detection ${SOURCE_FILES})
rosidl_get_typesupport_target(perception_typesupport_target
    ${PROJECT_NAME}_interfaces
    "rosidl_typesupport_cpp"
)
target_link_libraries(free_space_detection "${perception_typesupport_target}")

target_include_directories(free_space_detection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(free_space_detection PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++23

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# --------------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------------
ament_auto_package(INSTALL_TO_SHARE)