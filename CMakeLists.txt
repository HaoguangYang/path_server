cmake_minimum_required(VERSION 3.8)
project(path_server)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${ROS_VERSION} EQUAL 2)
  # find dependencies
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_add_library(path_server_lib SHARED
    src/bezier_interpolation.cpp
    src/path_manipulation.cpp
    src/path_server_utils.cpp
    src/path_server.cpp
    src/path_server_node.cpp
  )
  rclcpp_components_register_node(path_server_lib
    PLUGIN "path_server::PathServerNode"
    EXECUTABLE path_server_node
  )

  # export information to downstream packages
  ament_auto_package(
    CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
    INSTALL_TO_SHARE maps launch
  )

  if(AMENT_ENABLE_TESTING)
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
elseif(${ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
  )
  find_package(cmake_modules)
  find_package(Eigen REQUIRED)

  ## Declare a catkin package
  catkin_package()

  ## Build talker and listener
  include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

  add_executable(path_server
    src/bezier_interpolation.cpp
    src/path_manipulation.cpp
    src/path_server_utils.cpp
    src/path_server.cpp
    src/path_server_node_ros1.cpp
  )
  target_link_libraries(path_server ${catkin_LIBRARIES})
  add_dependencies(path_server
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  )

endif()
