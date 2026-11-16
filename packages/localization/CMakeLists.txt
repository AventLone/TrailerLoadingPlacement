cmake_minimum_required(VERSION 3.28)
project(mapping_localization LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
        $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    )
endif ()

# --------------------------------------------------------------------------------
# Find ROS2 dependencies
# --------------------------------------------------------------------------------
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies() # Automaticly find all the packages mentioned in 'package.xml'

set(COMMON_SOURCE_FILES
    source/GICP/GICP.cpp
    source/types/SparseVoxel.cpp
)

find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)

ament_auto_add_executable(localization
    source/localization.cpp
    source/nodes/Localization.cpp
    ${COMMON_SOURCE_FILES}
)

ament_auto_add_executable(mapping
    source/mapping.cpp
    source/nodes/Mapping.cpp
    ${COMMON_SOURCE_FILES}
)

foreach (target localization mapping)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )
    target_link_libraries(${target} Sophus::Sophus TBB::tbb)
    target_compile_features(${target} PUBLIC c_std_99 cxx_std_23)
endforeach ()

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# --------------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------------
ament_auto_package(INSTALL_TO_SHARE config)