cmake_minimum_required(VERSION 3.16)
project(hw_turjabot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  PythonLibs
  pybind11
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(/usr/local/include/python3.10)

## COMPILE
add_library(
  hw_turjabot
  SHARED
  hardware/turjabot_hardware.cpp
)
target_compile_features(hw_turjabot PUBLIC cxx_std_17)

target_include_directories(hw_turjabot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/hw_turjabot>
)


ament_target_dependencies(
  hw_turjabot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "HW_TURJABOT_BUILDING_DLL")


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hw_turjabot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/hw_turjabot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/hw_turjabot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/hw_turjabot
)
install(TARGETS hw_turjabot
  EXPORT export_hw_turjabot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_hw_turjabot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
