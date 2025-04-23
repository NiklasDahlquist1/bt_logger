cmake_minimum_required(VERSION 3.8)
project(bt_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_package()







install(TARGETS
  DESTINATION lib/${PROJECT_NAME})
  
  
install(DIRECTORY
#  launch
  DESTINATION share/${PROJECT_NAME})



# build msgs
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BtReturnStatus.msg"
  "msg/BtTransition.msg"
  "msg/Location.msg"
  "msg/LocationArray.msg"
  DEPENDENCIES 
  builtin_interfaces 
  rosidl_default_generators
  std_msgs
  nav_msgs
  geometry_msgs
 )

ament_export_dependencies(rosidl_default_runtime)



# Seems to be needed for humble?
#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)



# probably the recommended galactic way?
#rosidl_target_interfaces(node
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


  
ament_package()
