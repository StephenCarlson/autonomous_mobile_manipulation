cmake_minimum_required(VERSION 2.8.3)
project(boat_planner)

# add_definitions(-std=c++11) 
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) #-std=c++14 vs -std=gnu++14

# find_package(PCL REQUIRED)
# message("PCL Version:" ${PCL_VERSION})
# message("PCL Path:" ${PCL_DIR})


find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  # message_generation
  # nodelet
  roslaunch
  roscpp
  std_msgs
  # geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   DriveToTarget.srv
# )
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(

  CATKIN_DEPENDS 
    # message_generation 
    roscpp
    roslaunch
    std_msgs
  INCLUDE_DIRS
    include
  #  LIBRARIES boat_planner
#  DEPENDS system_lib
)



include_directories(include)
include_directories(SYSTEM 
include 
${catkin_INCLUDE_DIRS})


# # Library
add_library(MGI_BoatPlanner_lib src/boat_planner/MGI_BoatPlanner.cpp)
target_link_libraries(MGI_BoatPlanner_lib ${catkin_LIBRARIES})
# 
# # Standalone node
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node MGI_BoatPlanner_lib)
# ${PROJECT_NAME} ${PROJECT_NAME}_lib)
roslaunch_add_file_check(launch)




# Works?
# add_executable(boat_planner_node src/boat_planner_node.cpp)
# target_link_libraries(boat_planner_node ${catkin_LIBRARIES})

# add_dependencies(boat_planner_node boat_planner_generate_messages_cpp)

# install(DIRECTORY config launch scripts
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 
# install(TARGETS boat_planner_lib
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# 
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# 
# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})