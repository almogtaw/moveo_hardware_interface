cmake_minimum_required(VERSION 3.8)
project(moveo_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  moveo_hardware_interface
  SHARED
  src/moveo_hardware_interface.cpp
  src/arduino_comm.cpp
)
target_compile_features(moveo_hardware_interface PUBLIC cxx_std_17)
target_include_directories(moveo_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveo_hardware_interface>
  /usr/include
)
ament_target_dependencies(
  moveo_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(moveo_hardware_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MOVEO_HARDWARE_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface moveo_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/moveo_hardware_interface
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf description/rviz
#   DESTINATION share/moveo_hardware_interface
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/moveo_hardware_interface
# )
install(TARGETS moveo_hardware_interface
  EXPORT export_moveo_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_moveo_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
