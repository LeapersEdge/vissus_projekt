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
# cpp stuff

# find all .cpp files to compile
file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# compile and link cpp_exec
add_executable(cpp_exec "${MY_SOURCES}")
ament_target_dependencies(cpp_exec rclcpp std_msgs nav_msgs geometry_msgs)

# add includes for .cpp
target_include_directories(cpp_exec PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_exec PUBLIC c_std_99 cxx_std_17)

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

# install cpp exec
install(TARGETS
    cpp_exec
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
