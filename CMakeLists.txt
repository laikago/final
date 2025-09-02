cmake_minimum_required(VERSION 3.8)
project(csv_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CsvData.msg"
  DEPENDENCIES std_msgs
)

# 构建节点
add_executable(csv_publisher_node src/csv_publisher_node.cpp)
ament_target_dependencies(csv_publisher_node rclcpp std_msgs)

# 链接消息库
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(csv_publisher_node "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS
  csv_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
