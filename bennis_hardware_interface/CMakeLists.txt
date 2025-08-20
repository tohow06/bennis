cmake_minimum_required(VERSION 3.8)
project(bennis_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)


add_library(cobs STATIC
  third_party/cobs-c/cobs.c
)
target_include_directories(cobs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/cobs-c>
  $<INSTALL_INTERFACE:include>   # 若你要把 cobs.h 一起安裝可用
)


add_library(
  bennis_hardware_interface
  SHARED
  src/arduino_comms.cpp
  src/wheel.cpp
  src/bennis_hardware.cpp
)
target_include_directories(
  bennis_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  bennis_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib 
)

target_link_libraries(
  bennis_hardware_interface 
  cobs
)


# prevent pluginlib from using boost
target_compile_definitions(bennis_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface bennis_hardware_interface.xml)

install(
  TARGETS
  bennis_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config launch description
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  bennis_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
