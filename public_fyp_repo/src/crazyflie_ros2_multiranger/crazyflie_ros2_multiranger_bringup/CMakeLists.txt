cmake_minimum_required(VERSION 3.5)

project(crazyflie_ros2_multiranger_bringup)

find_package(ament_cmake REQUIRED)

# Install project launch and config files
install(DIRECTORY
  launch
  config

  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/follow_waypoints.py
  scripts/send_initialpose.py
  scripts/slam_toolbox_load_map.py
  scripts/navigate_to_pose.py
  scripts/planners.py
  scripts/compare_reports.py
  scripts/compare_reports_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()