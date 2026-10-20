cmake_minimum_required(VERSION 3.8)
project(bai05_turtlebot4)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config maps rviz worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  models/malevisitoronphone models/fire_extinguisher
  DESTINATION share/${PROJECT_NAME}/models)
install(FILES models/yolo_world_furniture.torchscript DESTINATION share/${PROJECT_NAME}/models)
install(PROGRAMS scripts/navigate_goals.py scripts/yolo_detector.py DESTINATION lib/${PROJECT_NAME})
ament_package()
