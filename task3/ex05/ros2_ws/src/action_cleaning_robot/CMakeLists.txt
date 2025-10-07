cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# List of interface files
set(action_files
  "action/CleaningTask.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES action_msgs
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/cleaning_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_server
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/cleaning_action_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_client
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
