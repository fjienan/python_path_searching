cmake_minimum_required(VERSION 3.8)
project(python_path_searching)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  path_searching/astar_planner_node.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  path_searching/omnidirectional_tracker_node.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  path_searching/pure_pursuit_node.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  path_searching/odom_simulator.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
