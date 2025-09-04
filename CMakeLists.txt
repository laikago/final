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

# -------------------------- 新增：OpenCV相关配置 --------------------------
# 1. 寻找OpenCV和cv_bridge依赖
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. 构建OpenCV可视化节点（替换原matplotlib节点，或新增）
add_executable(data_processor_node src/data_processor_node.cpp)

#
ament_target_dependencies(data_processor_node
  rclcpp 
  std_msgs 
  cv_bridge 
  OpenCV
  csv_publisher
)
#
# 3. 链接OpenCV库
target_link_libraries(data_processor_node
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}" # 复用消息类型支持目标
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
