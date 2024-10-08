cmake_minimum_required(VERSION 3.10)
project(eigen_learn)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 查找PCL库
find_package(PCL REQUIRED)

# 添加PCL库的头文件路径
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 自动查找 src 目录下的所有 .cpp 文件
file(GLOB SOURCES "src/*.cpp")

# 为每个 .cpp 文件生成一个可执行文件
foreach(SOURCE_FILE ${SOURCES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${FILE_NAME} ${SOURCE_FILE})
    target_link_libraries(${FILE_NAME} Eigen3::Eigen ${PCL_LIBRARIES})
endforeach()
