# 基于原始项目依赖的CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_pc)

SET(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

# 编译模式
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# 检查C++11支持
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# OpenCV 2.4.3 (原始项目要求)
find_package(OpenCV 2.4.3 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV 2.4.3 not found.")
endif()

# Eigen3
find_package(Eigen3 REQUIRED)

# Pangolin
find_package(Pangolin REQUIRED)

# PCL 1.7 (原始项目要求)
find_package(PCL 1.7 REQUIRED)

# OctoMap
FIND_PACKAGE(octomap REQUIRED)

# 包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/Examples/RGB-D/ty/common/include
    ${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/include
    ${OCTOMAP_INCLUDE_DIRS}
    /usr/local/include/DBoW2/include
    /usr/local/include/g2o
    /usr/local/include/ncnn
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# 库输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 创建共享库
add_library(${PROJECT_NAME} SHARED
    System.cc
    Tracking.cc
    LocalMapping.cc
    LoopClosing.cc
    ORBextractor.cc
    ORBmatcher.cc
    FrameDrawer.cc
    Converter.cc
    KeyFrame.cc
    Map.cc
    MapDrawer.cc
    Optimizer.cc
    PnPsolver.cc
    Frame.cc
    KeyFrameDatabase.cc
    Initializer.cc
    Viewer.cc
    pointcloudmapping.cc
    my_rgbd.cc
    ncnn_dect.cpp
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    /usr/local/lib/libDBoW2.so
    /usr/local/lib/libg2o_core.so
    /usr/local/lib/libncnn.a
    -fopenmp
)

# 可执行文件输出目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# RGB-D示例
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(ty_rgbd
    ../my_rgbd.cc
)
target_link_libraries(ty_rgbd ${PROJECT_NAME})
