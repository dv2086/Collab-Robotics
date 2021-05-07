cmake_minimum_required(VERSION 3.5)
project(plansys_multirobot)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)


add_library(plansys2_move_bt_node SHARED src/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(lift_action_node src/lift_action_node.cpp)
ament_target_dependencies(lift_action_node ${dependencies})

add_executable(drop_action_node src/drop_action_node.cpp)
ament_target_dependencies(drop_action_node ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  lift_action_node
  drop_action_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()