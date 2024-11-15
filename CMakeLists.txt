cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Export rosidl dependencies
ament_export_dependencies(rosidl_default_runtime)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
)

# Get typesupport targets for rosidl
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Add executables
add_executable(talker src/publisher_member_function.cpp)
target_link_libraries(talker ${cpp_typesupport_target})
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

add_executable(server_client src/server_client.cpp)
target_link_libraries(server_client ${cpp_typesupport_target})
ament_target_dependencies(server_client rclcpp)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Install executables
install(TARGETS
  talker
  listener
  server_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add GTest for test.cpp
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${cpp_typesupport_target})
    ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs tf2 tf2_ros)
  endif()
endif()

# Documentation generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
  set(DOXYGEN_DOT_TRANSPARENT YES)
else()
  message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif()

ament_package()
