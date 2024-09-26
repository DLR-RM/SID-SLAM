cmake_minimum_required(VERSION 2.8.9)
project (LCDetector)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-undef")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_definitions(-DSRRG_HBST_HAS_OPENCV)

include_directories(include
        thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${Python3_INCLUDE_DIRS})

add_library(LCDetector SHARED src/LCDetector.cpp src/symbol.cpp)
target_link_libraries(LCDetector
        ${OpenCV_LIBRARIES})


set(INCLUDE_INSTALL_DIR include CACHE STRING "")
set(LIB_INSTALL_DIR lib CACHE STRING "")
set(BIN_INSTALL_DIR bin CACHE STRING "")
set(CMAKE_INSTALL_DIR lib/LCDetector/cmake CACHE STRING "")
set(SYSCONFIG_INSTALL_DIR /etc/LCDetector CACHE STRING "")

install(
    TARGETS LCDetector 
    EXPORT LCDetector-exports
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

INSTALL (
        DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")

install(EXPORT LCDetector-exports
        FILE LCDetectorTargets.cmake
        NAMESPACE LCDetector::
        DESTINATION ${LIB_INSTALL_DIR}/LCDetector/cmake
)

include(CMakePackageConfigHelpers)

configure_package_config_file(cmake/LCDetectorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/LCDetector/LCDetectorConfig.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/LCDetector/cmake
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR CMAKE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/LCDetector/LCDetectorConfigVersion.cmake
        VERSION 0.0.0
        COMPATIBILITY SameMajorVersion )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/LCDetector/LCDetectorConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/LCDetector/LCDetectorConfigVersion.cmake
        DESTINATION ${LIB_INSTALL_DIR}/LCDetector/cmake )
