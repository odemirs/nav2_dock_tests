cmake_minimum_required(VERSION 3.20)
project(nav2_dock_tests)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
    nodes/dock_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    maps
    models
    params
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
