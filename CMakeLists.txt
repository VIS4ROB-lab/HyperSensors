cmake_minimum_required(VERSION 3.16)
project(hyper_sensors VERSION 23.05.09 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/HyperSensors LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(ament_cmake REQUIRED)
find_package(hyper_variables REQUIRED)
find_package(hyper_state REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/internal/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE_FILES})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

ament_target_dependencies(${PROJECT_NAME} hyper_variables hyper_state cv_bridge yaml-cpp)
ament_export_dependencies(hyper_variables hyper_state cv_bridge yaml-cpp)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    file(GLOB_RECURSE PROJECT_TEST_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/tests/*.hpp ${PROJECT_SOURCE_DIR}/tests/*.cpp)

    if (PROJECT_TEST_FILES)
        ament_add_gtest(${PROJECT_NAME}_tests ${PROJECT_TEST_FILES})
        target_include_directories(${PROJECT_NAME}_tests PUBLIC
                $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/tests/include>
                $<INSTALL_INTERFACE:include>
                )
        target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
    endif ()
endif ()

ament_package()
