# 设定CMake的最低版本要求为2.8.3
cmake_minimum_required(VERSION 2.8.3)

# 定义项目的名称为 plan_env
project(plan_env)

# 设置构建类型为Release模式
set(CMAKE_BUILD_TYPE "Release")

# 添加编译选项，使用C++11标准
ADD_COMPILE_OPTIONS(-std=c++11 )

# 添加编译选项，使用C++14标准（通常你只需要一个标准，这里可能是为了确保兼容性）
ADD_COMPILE_OPTIONS(-std=c++14 )

# 设置Release模式下的编译标志：优化级别为3，开启所有警告，生成调试信息
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找catkin构建系统所需的组件
find_package(catkin REQUIRED COMPONENTS
  roscpp                 # ROS C++客户端库
  rospy                  # ROS Python客户端库
  std_msgs               # 标准消息类型
  visualization_msgs     # 可视化消息类型
  cv_bridge              # OpenCV和ROS图像类型之间的桥梁
  message_filters        # 消息过滤器
)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 查找PCL（点云库）1.7版本
find_package(PCL 1.7 REQUIRED)

# 定义catkin包
catkin_package(
  INCLUDE_DIRS include     # 包含头文件目录
  LIBRARIES plan_env       # 库文件名称
  CATKIN_DEPENDS roscpp std_msgs # 依赖的catkin包
#  DEPENDS system_lib     # 其他系统库依赖（此行被注释掉了）
)

# 包含目录
include_directories(
    SYSTEM                # 标志着这些目录是系统头文件，避免生成不必要的警告
    include               # 包含当前项目的include目录
    ${catkin_INCLUDE_DIRS}# 包含catkin包的头文件目录
    ${Eigen3_INCLUDE_DIRS}# 包含Eigen3库的头文件目录
    ${PCL_INCLUDE_DIRS}   # 包含PCL库的头文件目录
    ${OpenCV_INCLUDE_DIRS}# 包含OpenCV库的头文件目录
)

# 指定链接库文件的目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加库文件plan_env及其对应的源文件
add_library(plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
)

# 指定链接库文件plan_env所需的依赖库
target_link_libraries(plan_env
    ${catkin_LIBRARIES}   # 链接catkin库
    ${PCL_LIBRARIES}      # 链接PCL库
)
