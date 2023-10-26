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

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

if(${ROS_VERSION} EQUAL 2)
  # find dependencies
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # main program
  ament_auto_add_library(path_server_lib SHARED
    src/bezier_interpolation.cpp
    src/path_manipulation.cpp
    src/path_server.cpp
    src/path_server_node.cpp
    src/path_data_extd.cpp
  )

  # linking GeographicLib as it is added with the custom cmake path
  target_link_libraries(path_server_lib
    ${GeographicLib_LIBRARIES}
  )

  rclcpp_components_register_node(path_server_lib
    PLUGIN "path_server::PathServerNode"
    EXECUTABLE path_server_node
  )

  # custom message types
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PathOffsetCommand.msg"
    DEPENDENCIES
      builtin_interfaces
      std_msgs
    ADD_LINTER_TESTS
  )

  if(NOT COMMAND rosidl_get_typesupport_target)
    function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
      if(NOT TARGET ${generate_interfaces_target})
        message(FATAL_ERROR
          "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
      endif()

      set(output_target "${generate_interfaces_target}__${typesupport_name}")

      if(NOT TARGET ${output_target})
        # CMake if() evaluates strings ending in `-NOTFOUND` as false
        set(output_target "${output_target}-NOTFOUND")
      endif()

      set("${var}" "${output_target}" PARENT_SCOPE)
    endfunction()
  endif()

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  # plugins
  ament_auto_add_library(path_server_plugins SHARED
    src/path_properties/active_segment.cpp
    src/path_properties/curvature.cpp
    src/path_properties/curve_length.cpp
    src/path_properties/lateral_offset.cpp
    src/path_properties/nop.cpp
  )
  target_link_libraries(path_server_plugins "${cpp_typesupport_target}")
  pluginlib_export_plugin_description_file(path_server_overhaul plugins.xml)

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
    pluginlib
  )
  find_package(cmake_modules)
  find_package(Eigen REQUIRED)

  ## Declare a catkin package
  catkin_package()

  ## Build talker and listener
  include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

  add_executable(path_server_node
    src/bezier_interpolation.cpp
    src/path_manipulation.cpp
    src/path_server.cpp
    src/path_server_node_ros1.cpp
    src/path_data_extd.cpp
  )

  target_link_libraries(path_server_node ${catkin_LIBRARIES})

  # linking GeographicLib as it is added with the custom cmake path
  target_link_libraries(path_server_node
    ${GeographicLib_LIBRARIES}
  )

  add_dependencies(path_server_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  )

endif()
