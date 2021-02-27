cmake_minimum_required(VERSION 3.5)
project(sub_behavior)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(sub_behavior_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)


set(dependencies
        rclcpp
        rclcpp_action
        behaviortree_cpp_v3
        geometry_msgs
        nav_msgs
        std_msgs
        std_srvs
        behavior_tree
        sub_behavior_interfaces
        )

add_library(move_distance_bt_node SHARED plugins/action/move_distance.cpp)
list(APPEND plugin_libs move_distance_bt_node)

add_library(snap_photo_bt_node SHARED plugins/action/snap_photo.cpp)
list(APPEND plugin_libs snap_photo_bt_node)

add_library(random_displacement_bt_node SHARED plugins/action/random_displacement.cpp)
list(APPEND plugin_libs random_displacement_bt_node)

add_library(random_failure_bt_node SHARED plugins/action/random_failure.cpp)
list(APPEND plugin_libs random_failure_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY launch trees
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
        examples/scripts/move_distance_action.py
        examples/scripts/snap_photo_service.py
        DESTINATION lib/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
