cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(ORB_SLAM2_SSD_Semantic VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译模式
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 显示编译模式信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

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
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

# 设置模块路径
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 寻找依赖库
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)

# 可选：查找OctoMap
find_package(octomap QUIET)
if(octomap_FOUND)
    message(STATUS "Found OctoMap")
    add_definitions(-DUSE_OCTOMAP)
else()
    message(STATUS "OctoMap not found, building without OctoMap support")
endif()

# 设置输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(octomap_FOUND)
    include_directories(${OCTOMAP_INCLUDE_DIRS})
endif()

# 添加定义
add_definitions(${PCL_DEFINITIONS})

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 选项：构建哪个版本
option(BUILD_SRC "Build src version" ON)
option(BUILD_PERFECT "Build perfect version" ON)
option(BUILD_REALTIME_DETECT "Build realtime detection" ON)

# 添加子目录
if(BUILD_SRC)
    add_subdirectory(src)
endif()

if(BUILD_PERFECT)
    add_subdirectory(perfect)
endif()

if(BUILD_REALTIME_DETECT)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/realtime_dect_loc/CMakeLists.txt")
        add_subdirectory(realtime_dect_loc)
    endif()
endif()

# 安装配置
install(DIRECTORY include/ DESTINATION include/orb_slam2_ssd_semantic)
install(DIRECTORY docker/ DESTINATION share/orb_slam2_ssd_semantic/docker)

# 打印配置信息
message(STATUS "")
message(STATUS "ORB_SLAM2_SSD_Semantic Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "  PCL version: ${PCL_VERSION}")
if(octomap_FOUND)
    message(STATUS "  OctoMap: Found")
else()
    message(STATUS "  OctoMap: Not found")
endif()
message(STATUS "  Build src: ${BUILD_SRC}")
message(STATUS "  Build perfect: ${BUILD_PERFECT}")
message(STATUS "  Build realtime detection: ${BUILD_REALTIME_DETECT}")
message(STATUS "")
