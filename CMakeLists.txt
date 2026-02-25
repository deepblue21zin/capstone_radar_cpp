cmake_minimum_required(VERSION 3.16)
project(RadarTracking CXX)

# ============================================================
# Project Configuration
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
# Compiler Flags
# ============================================================

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Optimization flags
    if(MSVC)
        add_compile_options(/O2 /Oi /Ot /GL)
        add_link_options(/LTCG)
    else()
        # GCC/Clang
        add_compile_options(-O3 -march=native -ffast-math)
        add_compile_options(-flto)  # Link-time optimization
    endif()
    
    add_compile_definitions(NDEBUG)
    message(STATUS "Optimization: -O3 -march=native")
    
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(MSVC)
        add_compile_options(/Od /Zi)
    else()
        add_compile_options(-O0 -g -Wall -Wextra -Wpedantic)
    endif()
    message(STATUS "Debug: Full symbols, no optimization")
endif()

# ============================================================
# Find Required Packages
# ============================================================

# Eigen (required)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "✓ Eigen3 found at ${EIGEN3_INCLUDE_DIR}")

# fmt (optional, for logging)
find_package(fmt QUIET)
if(fmt_FOUND)
    message(STATUS "✓ fmt found (logging enabled)")
else()
    message(STATUS "⊘ fmt not found (using iostream)")
endif()

# OpenCV (optional, for visualization)
find_package(OpenCV 4.0 QUIET COMPONENTS core)
if(OpenCV_FOUND)
    message(STATUS "✓ OpenCV found (visualization enabled)")
    add_compile_definitions(ENABLE_VISUALIZATION)
else()
    message(STATUS "⊘ OpenCV not found (visualization disabled)")
endif()

# GTest (optional, for testing)
find_package(GTest QUIET)
if(GTest_FOUND)
    message(STATUS "✓ GTest found (testing enabled)")
    enable_testing()
else()
    message(STATUS "⊘ GTest not found (testing disabled)")
endif()

# ============================================================
# Main Library: radar_tracking
# ============================================================

add_library(radar_tracking SHARED
    src/kalman_filter.cpp
    src/clusterer.cpp
    src/hungarian_matcher.cpp
    src/tracker.cpp
    src/preprocessor.cpp
    src/radar_tracker.cpp
)

target_include_directories(radar_tracking PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(radar_tracking PUBLIC
    Eigen3::Eigen
)

if(fmt_FOUND)
    target_link_libraries(radar_tracking PRIVATE fmt::fmt)
    target_compile_definitions(radar_tracking PRIVATE HAVE_FMT)
endif()

if(OpenCV_FOUND)
    target_link_libraries(radar_tracking PRIVATE ${OpenCV_LIBRARIES})
endif()

# ============================================================
# Main Executable
# ============================================================

add_executable(radar_tracking_main src/main.cpp)
target_link_libraries(radar_tracking_main PRIVATE radar_tracking)

# ============================================================
# Unit Tests
# ============================================================

if(GTest_FOUND)
    add_executable(test_kalman_filter
        test/test_kalman_filter.cpp
    )
    target_link_libraries(test_kalman_filter PRIVATE
        radar_tracking
        GTest::gtest_main
    )
    add_test(NAME kalman_filter COMMAND test_kalman_filter)
    
    add_executable(test_dbscan
        test/test_dbscan.cpp
    )
    target_link_libraries(test_dbscan PRIVATE
        radar_tracking
        GTest::gtest_main
    )
    add_test(NAME dbscan COMMAND test_dbscan)
    
    add_executable(test_integration
        test/test_integration.cpp
    )
    target_link_libraries(test_integration PRIVATE
        radar_tracking
        GTest::gtest_main
    )
    add_test(NAME integration COMMAND test_integration)
    
    message(STATUS "Tests: build with `cmake --build . --target test`")
endif()

# ============================================================
# Installation
# ============================================================

install(TARGETS radar_tracking radar_tracking_main
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config/
    DESTINATION etc/radar_tracking
)

# ============================================================
# Summary
# ============================================================

message(STATUS "")
message(STATUS "=== RadarTracking Build Summary ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "")
message(STATUS "Libraries:")
message(STATUS "  ✓ Eigen3")
if(fmt_FOUND)
    message(STATUS "  ✓ fmt")
else()
    message(STATUS "  ⊘ fmt")
endif()
if(OpenCV_FOUND)
    message(STATUS "  ✓ OpenCV (visualization)")
else()
    message(STATUS "  ⊘ OpenCV (no visualization)")
endif()
if(GTest_FOUND)
    message(STATUS "  ✓ GTest (unit testing)")
else()
    message(STATUS "  ⊘ GTest (no unit tests)")
endif()
message(STATUS "")
message(STATUS "Build commands:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake -DCMAKE_BUILD_TYPE=Release ..")
message(STATUS "  cmake --build . -j$(nproc)")
message(STATUS "  ./radar_tracking_main")
message(STATUS "")
