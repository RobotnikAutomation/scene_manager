cmake_minimum_required(VERSION 3.0.2)
project(scene_manager)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
  roscpp
  std_msgs
  tf2_ros
  tf2
  moveit_msgs
  scene_manager_msgs
  tf_conversions
  eigen_conversions
  moveit_visual_tools
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp scene_manager_msgs
 DEPENDS
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/scene_manager.cpp
  src/object_builder.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  PATTERN "setup_assistant.launch" EXCLUDE)

#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)