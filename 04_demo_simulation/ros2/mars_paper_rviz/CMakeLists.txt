cmake_minimum_required(VERSION 3.16)
project(mars_paper_rviz)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/paper_rviz_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME paper_rviz_node)
install(PROGRAMS scripts/live_lidar_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME live_lidar_node)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../fig8_fig9.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../online_mission.py
  DESTINATION share/${PROJECT_NAME}/core)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
