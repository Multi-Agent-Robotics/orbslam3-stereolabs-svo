cmake_minimum_required (VERSION 3.16.3 FATAL_ERROR)

project (orbslam3-stereolabs-svo
    DESCRIPTION "ORB-SLAM3 with Stereolabs ZED SVO file"
    LANGUAGES CXX)

include (cmake/ccache.cmake)

# targets
add_executable (orbslam3-stereolabs-svo "")

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
install (FILES ${CMAKE_BINARY_DIR}/compile_commands.json DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Release)
    message (STATUS "Build type not specified: Use Release by default")
endif ()

use_ccache_if_available ()

find_package (orbslam3 REQUIRED)
find_package (ZED 3 REQUIRED)
find_package (CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package (OpenCV REQUIRED)

# option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)

# if (${FORCE_COLORED_OUTPUT})
# if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
# add_compile_options (-fdiagnostics-color=always)
# elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
# add_compile_options (-fcolor-diagnostics)
# endif ()
# endif ()
target_sources (orbslam3-stereolabs-svo
    PRIVATE
        src/main.cpp
)

target_link_libraries (orbslam3-stereolabs-svo
    PRIVATE
    orbslam3
    ${ZED_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY}
)

target_include_directories (orbslam3-stereolabs-svo
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

message (STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")

target_compile_options (orbslam3-stereolabs-svo PRIVATE
    -Wall
    -Wextra

    # if the compiler is clang
    $<$<STREQUAL:${CMAKE_CXX_COMPILER_ID},Clang>:-fcolor-diagnostics>

    # if the compiler is gcc
    $<$<STREQUAL:${CMAKE_CXX_COMPILER_ID},GNU>:-fdiagnostics-color=always>
)

target_compile_features (orbslam3-stereolabs-svo
    PRIVATE
        cxx_std_20
)

export (TARGETS orbslam3-stereolabs-svo
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake
)

# make the library available in the users local cmake registry at ~/.cmake/packages/${PROJECT_NAME}
set (CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export (PACKAGE ${PROJECT_NAME})
message (STATUS "Exported package ${PROJECT_NAME}")
message (STATUS "Find it with `find_package(${PROJECT_NAME})`.")
message (STATUS "A link to it will be at ~/.cmake/packages/${PROJECT_NAME}")