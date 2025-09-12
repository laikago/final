cmake_minimum_required(VERSION 3.8)
project(data_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(csv_publisher REQUIRED)

find_package(rosidl_default_generators REQUIRED)#发布新增

#发布新增
# 声明消息文件
set(msg_files
  "msg/Output.msg"  # 确认此文件存在
 # DEPENDENCIES std_msgs
)

# 生成消息接口（关键步骤）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
# -------------------------- 新增：OpenCV相关配置 --------------------------
# 1. 寻找OpenCV和cv_bridge依赖
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS}) 
# 头文件路径：指定include目录（必须添加，否则找不到自定义头文件）
include_directories(
  include
)
# CMakeLists.txt 中
find_package(yaml-cpp REQUIRED)  # 查找YAML-CPP

# 生成可执行文件（将主文件和类实现文件一起编译）
#add_executable(armor_node 
#  src/data_processor_node.cpp 
#  src/ekf.cpp
#)


# 2. 构建OpenCV可视化节点（替换原matplotlib节点，或新增）
add_executable(data_processor_node src/data_processor_node.cpp src/ekf.cpp)

#
ament_target_dependencies(data_processor_node
  rclcpp 
  std_msgs 
  cv_bridge 
  OpenCV
  csv_publisher
  yaml-cpp
)
#
# 3. 链接OpenCV库
target_link_libraries(data_processor_node
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}" # 复用消息类型支持目标
   ${YAML_CPP_LIBRARIES}  # 链接 YAML 库
)

target_include_directories(data_processor_node PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}  # YAML 头文件路径
  ${EIGEN3_INCLUDE_DIRS}    # Eigen 头文件路径
  ../include                 # 若 ekf.h 在 include 目录
)


# 链接消息接口（否则节点找不到消息类型）
rosidl_target_interfaces(data_processor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
# 4. 安装OpenCV可视化节点
install(TARGETS
  data_processor_node  # 新增：OpenCV可视化节点
  DESTINATION lib/${PROJECT_NAME})
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
