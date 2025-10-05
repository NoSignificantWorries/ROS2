cmake_minimum_required(VERSION 3.8)
project(service_full_name)

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
set(SRV_FILES
  "srv/SummFullName.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/service_full_name/service_name.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_name
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/service_full_name/client_name.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME client_name
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
