# 设置 CMake 最低版本
cmake_minimum_required(VERSION 3.10)

# 定义项目名称
project(eigen_learn)

# 设置 C++ 标准为 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 查找PCL库，包含visualization模块
find_package(PCL REQUIRED COMPONENTS common io visualization)

# 添加PCL库的头文件路径
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 自动查找 src 目录下的所有 .cpp 文件
file(GLOB SOURCES "src/*.cpp")

# 为每个 .cpp 文件生成一个可执行文件，保留文件名中的小数点
foreach(SOURCE_FILE ${SOURCES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME)
    
    # 手动去掉扩展名 .cpp
    string(REPLACE ".cpp" "" EXEC_NAME ${FILE_NAME})
    
    # 检查目标是否已存在，避免重复定义
    if (NOT TARGET ${EXEC_NAME})
        add_executable(${EXEC_NAME} ${SOURCE_FILE})
        target_link_libraries(${EXEC_NAME} Eigen3::Eigen ${PCL_LIBRARIES})
    endif()
endforeach()

# 抑制开发者警告
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)

