cmake_minimum_required(VERSION 3.8)
project(my_robot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(odrive_mit_example REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)

add_library(leg_impedance_controller SHARED
  src/leg_impedance_controller.cpp
)
target_include_directories(leg_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${odrive_mit_example_INCLUDE_DIRS}
)
target_link_libraries(leg_impedance_controller
  rclcpp::rclcpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  ${odrive_mit_example_LIBRARIES}
  joint_limits::joint_limits
  ${urdf_LIBRARIES}
)

# Install the library
install(
  TARGETS leg_impedance_controller
  EXPORT export_leg_impedance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(
  FILES my_robot_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export the plugin
ament_export_libraries(leg_impedance_controller)
ament_export_dependencies(
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface my_robot_controllers.xml)

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
