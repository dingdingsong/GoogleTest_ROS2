cmake_minimum_required(VERSION 3.5)
project(my_ros2_node)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

set(GMock_DIR "/usr/src/gmock")
set(GTest_DIR "/usr/src/gtest")


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Threads REQUIRED)

# Set up include directories
message(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/my_ros2_node
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${example_interfaces_INCLUDE_DIRS}
  include(GoogleTest)
  include(GMock)
)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()


# Add tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(my_ros2_node test/test_my_node.cpp src/my_node.cpp)

  target_link_libraries(my_ros2_node
    # my_ros2_node_lib
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${example_interfaces_LIBRARIES}
    GTest::gmock_main
    GTest::gtest_main
    pthread
  )
  ament_add_test(test_my_node COMMAND test_my_node
)
endif()
install(TARGETS
  my_ros2_node
  # my_ros2_node_lib
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
