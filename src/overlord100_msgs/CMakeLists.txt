cmake_minimum_required(VERSION 3.8)
project(overlord100_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
# 取消注释以下部分以手动填写
# 进一步的依赖项。
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelsData.msg"
  DEPENDENCIES  # 添加上述消息依赖的包，在本例中为Sphere.msg的geometry_msgs
)

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
