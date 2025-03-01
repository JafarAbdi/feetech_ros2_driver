cmake_minimum_required(VERSION 3.16)
project(feetech_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Werror
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wsign-conversion
    -Winit-self
    -Wredundant-decls)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(tl-expected QUIET)
if(NOT tl-expected_FOUND)
  # If the first name is not found, try the second name
  find_package(tl_expected QUIET)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

if(NOT tl-expected_FOUND AND NOT tl_expected_FOUND)
  # Neither name was found
  message(
    FATAL_ERROR
      "Could not find the package with either name 'tl-expected' or 'tl_expected'"
  )
endif()
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(range-v3 REQUIRED)
# Find LibSerial package
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

# Get version from version.hpp
file(READ include/feetech_driver/version.hpp version_hpp)
if(NOT version_hpp MATCHES
   "FEETECH_DRIVER_VERSION ([0-9]+)([0-9][0-9])([0-9][0-9])")
  message(FATAL_ERROR "Cannot get FEETECH_DRIVER_VERSION from version.hpp")
endif()
# Use math to skip leading zeros if any.
math(EXPR CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_MATCH_1})
math(EXPR CPACK_PACKAGE_VERSION_MINOR ${CMAKE_MATCH_2})
math(EXPR CPACK_PACKAGE_VERSION_PATCH ${CMAKE_MATCH_3})
# cmake-lint: disable=C0103
set(${PROJECT_NAME}_VERSION
    "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}"
)
message(STATUS "${PROJECT_NAME} version: ${${PROJECT_NAME}_VERSION}")

add_library(serial_port src/serial_port.cpp)
target_compile_features(serial_port PRIVATE cxx_std_20)
target_include_directories(
  serial_port PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_include_directories(serial_port SYSTEM PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(serial_port PUBLIC fmt::fmt spdlog::spdlog tl::expected
                                         ${SERIAL_LIBRARIES})
set_target_properties(
  serial_port PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                         SOVERSION ${CPACK_PACKAGE_VERSION_MAJOR})

add_library(communication_protocol src/communication_protocol.cpp)
target_compile_features(communication_protocol PRIVATE cxx_std_20)
target_include_directories(
  communication_protocol
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(communication_protocol PUBLIC serial_port)
set_target_properties(
  communication_protocol PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                                    SOVERSION ${CPACK_PACKAGE_VERSION_MAJOR})

add_executable(demo examples/demo.cpp)
target_compile_features(demo PRIVATE cxx_std_20)
target_link_libraries(demo serial_port communication_protocol)

include(CMakePackageConfigHelpers)
# Generate the version, config and target files into the build directory.
write_basic_package_version_file(
  ${PROJECT_NAME}-config-version.cmake
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

include(GNUInstallDirs)
install(
  TARGETS serial_port communication_protocol
  COMPONENT ${PROJECT_NAME}-core
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(TARGETS demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Use a namespace because CMake provides better diagnostics for namespaced
# imported targets.
export(
  TARGETS serial_port communication_protocol
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}-targets.cmake)

# Install version, config and target files.
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  COMPONENT ${PROJECT_NAME}-core)
install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  COMPONENT ${PROJECT_NAME}-core
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
