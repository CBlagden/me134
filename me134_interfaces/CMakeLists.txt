cmake_minimum_required(VERSION 3.5)
project(me134_interfaces)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateStamped.msg"
  "msg/JointCmdStamped.msg"
  "msg/PosCmdStamped.msg"
  "msg/PoseTwoStamped.msg"
  "msg/SongMsg.msg"
  "srv/NoteCmdStamped.srv"
  "srv/PosCmdStamped.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
