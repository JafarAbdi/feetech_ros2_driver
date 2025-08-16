cmake_minimum_required(VERSION 3.16)
project(feetech_ros2_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wsign-conversion
    -Winit-self
    -Wredundant-decls)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

add_subdirectory(feetech_driver)

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(range-v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Set ${PROJECT_NAME}_VERSION
ament_package_xml()

add_library(${PROJECT_NAME} src/feetech_ros2_driver.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle
                          lifecycle_msgs hardware_interface pluginlib)
target_link_libraries(${PROJECT_NAME} communication_protocol
                      serial_port)

pluginlib_export_plugin_description_file(
  hardware_interface feetech_ros2_control_plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface pluginlib)

ament_generate_version_header(${PROJECT_NAME} HEADER_PATH
                              ${PROJECT_NAME}/version.hpp)

ament_package()
