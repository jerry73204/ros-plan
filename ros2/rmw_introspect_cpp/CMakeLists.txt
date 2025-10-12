cmake_minimum_required(VERSION 3.8)
project(rmw_introspect_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw_dds_common REQUIRED)

# RMW introspect library
add_library(${PROJECT_NAME} SHARED
  src/rmw_init.cpp
  src/data.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rcutils
  rmw
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_cpp
  rmw_dds_common
)

# Export library
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rcutils
  rmw
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_cpp
  rmw_dds_common
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_init test/test_init.cpp)
  target_link_libraries(test_init ${PROJECT_NAME})
  ament_target_dependencies(test_init rcutils rmw)

  ament_add_gtest(test_data test/test_data.cpp)
  target_link_libraries(test_data ${PROJECT_NAME})
  ament_target_dependencies(test_data rcutils)
endif()

ament_package()
