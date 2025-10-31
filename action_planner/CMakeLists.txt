cmake_minimum_required(VERSION 3.5)
project(action_planner)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(harpia_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclpy
    geometry_msgs
    lifecycle_msgs
    std_srvs
    harpia_msgs
)

install(DIRECTORY solver scripts DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/action_planner.py
  scripts/lifecycle_manager.py
  scripts/mission_controller.py
  scripts/move_abaixo.py
  scripts/move_acima.py
  scripts/entrar.py
  scripts/sair.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()