# Check CMake version.
cmake_minimum_required(VERSION 3.16)

# Includes.
include(FetchContent)

# Define projects.
project(HyperSensors VERSION 23.01.19 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/HyperSensors LANGUAGES CXX)

# Set properties.
if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED YES)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -fconcepts -Wall -Wextra -Wnon-virtual-dtor -pedantic")
    option(HYPER_BUILD_WITH_CERES "Build with Ceres." FALSE)
    option(HYPER_BUILD_TESTS "Build tests." TRUE)
endif ()

# Define libraries and aliases.
file(GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/internal/*.hpp ${PROJECT_SOURCE_DIR}/internal/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE_FILES})
add_library(Hyper::Sensors ALIAS ${PROJECT_NAME})

# Fetch (external) content.
FetchContent_Declare(HyperState GIT_REPOSITORY https://github.com/VIS4ROB-lab/HyperState.git GIT_TAG 257d30dd289c4ae4b4f37bcc6bb421eb175ba2ff)
FetchContent_MakeAvailable(HyperState)

# Find Eigen target if not present.
if (NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED NO_MODULE)
endif ()

# Find GTest target if not present.
if (NOT TARGET GTest::GTest)
    find_package(GTest REQUIRED)
endif ()

# Find glog target if not present.
if (NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif ()

# Find cv_bridge.
if (NOT TARGET cv_bridge::cv_bridge)
    find_package(cv_bridge REQUIRED)
endif ()

# Find yaml-cpp.
if (NOT TARGET yaml-cpp)
    find_package(yaml-cpp REQUIRED)
endif ()

if (HYPER_BUILD_WITH_CERES)
    # Find Ceres target if not present.
    if (NOT TARGET Ceres::ceres)
        find_package(Ceres REQUIRED)
    endif ()
endif ()

# Add includes and link libraries.
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} PUBLIC Hyper::State PUBLIC cv_bridge::cv_bridge PUBLIC yaml-cpp)

# Build tests.
if (HYPER_BUILD_TESTS)
    add_subdirectory(${PROJECT_SOURCE_DIR}/apps/tests)
endif (HYPER_BUILD_TESTS)
