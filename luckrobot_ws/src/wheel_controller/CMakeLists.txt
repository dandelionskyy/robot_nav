cmake_minimum_required(VERSION 3.8)
project(wheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 寻找依赖包 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# ==================== 编译节点可执行文件 ====================
# 导航调度节点 (航点导航)
add_executable(nav_manager_node src/nav_manager_node.cpp)

# ==================== 链接库与依赖项 ====================
ament_target_dependencies(nav_manager_node
  rclcpp
  std_msgs
  rclcpp_action
  nav2_msgs
)

# ==================== 安装规则 ====================
install(
  TARGETS
    nav_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
