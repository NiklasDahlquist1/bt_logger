cmake_minimum_required(VERSION 3.8)
project(bt_ros_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#add_compile_options(-std=c++17)


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(std_srvs REQUIRED)
#find_package(sensor_msgs REQUIRED)


find_package(behaviortree_cpp_v3 REQUIRED)


#find_package(tf2 REQUIRED)
find_package(bt_msgs REQUIRED)


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


ament_export_targets(${PROJECT_NAME})
ament_export_include_directories(include)
#ament_export_dependencies()

ament_package()



add_library(${PROJECT_NAME} 
  STATIC
  src/bt_ros_logger.cpp
#  ${BEHAVIORS_CPP}
)


ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    #auction_msgs
    std_msgs
    behaviortree_cpp_v3
    #geometry_msgs
    #nav_msgs
    #std_srvs
    #sensor_msgs
    #tf2
    bt_msgs

  )






install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)



