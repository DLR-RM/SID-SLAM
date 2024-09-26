cmake_minimum_required(VERSION 3.16)
project(idnav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -pthread")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS eigen opencv)

# Eigen
find_package( Eigen3 REQUIRED )
message(STATUS "Eigen3_DIR: ${Eigen3_DIR}")

# OpenCV
find_package( OpenCV 4 REQUIRED)
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
if( OpenCV_FOUND )
    # Additional Include Directories
    message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    # Additional Dependencies
    message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
endif()

# Pangolin
find_package(Pangolin REQUIRED)
message(STATUS "Pangolin_INCLUDE_DIRS: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")

# Ceres
find_package( Ceres REQUIRED )
message(STATUS "Ceres_DIR: ${Ceres_DIR}")

# Visual LCD
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/visual-lcd)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/visual-lcd/include)

# Fast Akaze
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/fast_akaze/fast_akaze)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/fast_akaze)

# Set IDNav library
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
        utils/file_functions.cpp
        utils/SequenceSettings.cpp
        utils/SystemSettings.cpp

        classes/Pt/Pt.cpp
        classes/Pose.cpp
        classes/Camera/Camera.cpp
        classes/Camera/camPinhole.cpp
        classes/Camera/camPinholeDist.cpp
        classes/Frame/Frame.cpp

        classes/informationFunctions.cpp

        classes/Tracker/Tracker.cpp
        classes/PointSelector/PointSelector.cpp

        classes/FrontEnd/trackingFrontEnd.cpp
        classes/backEnd.cpp
        classes/Visualizer.cpp

        classes/SID_SLAM.cpp

        classes/Tracker/RIA_CeresTracker.cpp

        classes/PointSelector/pointSelector_features.cpp
        classes/PointSelector/pointSelector_hgp.cpp
        classes/Tracker/CeresTracker.cpp
        classes/PoseGraphOptimizer.cpp
        classes/Optimizer/OptimizerCeres.cpp
        classes/Pt/perspectiveDeformation.cpp
        classes/Pt/ptJacobians.cpp
        classes/FrontEnd/informationFrontEnd.cpp
        classes/FrontEnd/frontEnd.cpp
        classes/Frame/ImageFunctions.cpp
        classes/FrontEnd/emergencyTracking.cpp
        classes/FrontEnd/loopClosure.cpp
        classes/Optimizer/DepthOptimizer.cpp)


    target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        akaze
        LCDetector
)

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D/TUM)
add_executable(rgbdTUM ${PROJECT_SOURCE_DIR}/Examples/RGB-D/TUM/rgbdTUM.cpp)
target_link_libraries(rgbdTUM ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D/ETH)
add_executable(rgbdETH ${PROJECT_SOURCE_DIR}/Examples/RGB-D/ETH/rgbdETH.cpp)
target_link_libraries(rgbdETH ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D/MT)
add_executable(rgbdMT ${PROJECT_SOURCE_DIR}/Examples/RGB-D/MT/rgbdMT.cpp)
target_link_libraries(rgbdMT ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D/NUIM)
add_executable(rgbdNUIM ${PROJECT_SOURCE_DIR}/Examples/RGB-D/NUIM/rgbdNUIM.cpp)
target_link_libraries(rgbdNUIM ${PROJECT_NAME})
