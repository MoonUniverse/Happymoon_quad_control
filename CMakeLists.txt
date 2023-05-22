cmake_minimum_required(VERSION 3.8)
project(happymoon_quad_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)


include_directories(
  include/happymoon_quad_control
)

ament_auto_add_library(
  happymoon_control
  src/happymoon_control.cpp
  src/geometry_eigen_conversions.cpp
  src/math_common.cpp
)

ament_auto_add_executable(happymoon_quad_control_node
  src/happymoon_quad_control_node.cpp
)

target_link_libraries(happymoon_quad_control_node
  happymoon_control
)

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

install(TARGETS
  happymoon_quad_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
