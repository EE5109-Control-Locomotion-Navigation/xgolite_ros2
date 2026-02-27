cmake_minimum_required(VERSION 3.5)
project(xgo2_ros)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/xgo_bt_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME xgo2_ros_node
)
install(PROGRAMS
  src/imu_to_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_to_odom_node
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/config.json
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
