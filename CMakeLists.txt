cmake_minimum_required(VERSION 3.8)
project(gazebo_distance_sensor_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)


set(IGN_CMAKE_VER 3)
set(IGN_COMMON_VER 3)
set(IGN_GAZEBO_VER 6)
find_package(gz-cmake${IGN_CMAKE_VER} REQUIRED)


gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim7 REQUIRED)



add_library(DistanceSensor SHARED src/distance_sensor_plugin.cpp)
target_include_directories(DistanceSensor  PRIVATE include)
set_property(TARGET DistanceSensor  PROPERTY CXX_STANDARD 17)

target_link_libraries(DistanceSensor 
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim7::gz-sim7
)



install(TARGETS DistanceSensor
  DESTINATION lib/${PROJECT_NAME}
)

configure_file("hooks/garden_environment.sh.in" "hooks/garden_environment.sh")

ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/hooks/garden_environment.sh")


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
