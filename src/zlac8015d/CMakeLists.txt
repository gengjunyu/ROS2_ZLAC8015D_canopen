cmake_minimum_required(VERSION 3.8)
project(zlac8015d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
# 取消注释以下部分以手动填写
# 进一步的依赖项。
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(overlord100_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motors_driver_node src/motors_driver_node.cpp)
target_include_directories(motors_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motors_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(encoders_node src/encoders_node.cpp)

add_executable(cmd_vel_to_wheels_node src/cmd_vel_to_wheels_node.cpp)

ament_target_dependencies(motors_driver_node encoders_node cmd_vel_to_wheels_node rclcpp can_msgs overlord100_msgs geometry_msgs)

install(TARGETS motors_driver_node cmd_vel_to_wheels_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行跳过检查版权的linter
  # 当版权和许可证添加到所有源文件时，注释掉此行
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下行跳过cpplint（仅在git仓库中工作）
  # 当此包在git仓库中且
  # 版权和许可证添加到所有源文件时，注释掉此行
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
