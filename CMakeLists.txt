cmake_minimum_required(VERSION 3.8)
project(vissus_projekt)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

#------------------------------------------
#------------------------------------------
#------------------------------------------

# msg stuff
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdometryArray.msg"
  DEPENDENCIES std_msgs nav_msgs
)

# cpp stuff

# find all .cpp files to compile
file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# compile and link boid_control
add_executable(boid_control "${MY_SOURCES}")
ament_target_dependencies(boid_control rclcpp std_msgs nav_msgs geometry_msgs)

# compile and link goal_pub
add_executable(goal_pub "${MY_SOURCES}")
ament_target_dependencies(goal_pub rclcpp std_msgs nav_msgs geometry_msgs)

# add includes for .cpp
target_include_directories(boid_control PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(boid_control PUBLIC c_std_99 cxx_std_17)

# add includes for .cpp
target_include_directories(goal_pub PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(goal_pub PUBLIC c_std_99 cxx_std_17)

#------------------------------------------
#------------------------------------------
#------------------------------------------
# python stuff

# ament_python_install_package(${PROJECT_NAME})

# find all .py files
file(GLOB_RECURSE MY_SCRIPTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
)

#------------------------------------------
#------------------------------------------
#------------------------------------------
# install stuff

# install boid_control
install(TARGETS
    boid_control
    DESTINATION lib/${PROJECT_NAME}
)

# install goal_pub
install(TARGETS
    goal_pub
    DESTINATION lib/${PROJECT_NAME}
)

# install py scripts
install(PROGRAMS
  "${MY_SCRIPTS}" 
  DESTINATION lib/${PROJECT_NAME}
)

# install launch scripts
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
