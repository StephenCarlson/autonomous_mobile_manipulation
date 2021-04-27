
# Udacity Course Leftovers
# project(ball_chaser)
# add_definitions(-std=c++11) 
# add_compile_options(-std=c++11)
# 
# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   roscpp
#   std_msgs
# )
# 
# add_service_files(
#   FILES
#   DriveToTarget.srv
# )
# 
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
# 
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES ball_chaser
# #  CATKIN_DEPENDS message_generation roscpp std_msgs
# #  DEPENDS system_lib
# )
# 
# add_executable(drive_bot src/drive_bot.cpp)
# add_executable(process_image src/process_image.cpp)
# target_link_libraries(drive_bot ${catkin_LIBRARIES})
# target_link_libraries(process_image ${catkin_LIBRARIES})
# add_dependencies(drive_bot ball_chaser_generate_messages_cpp)
# add_dependencies(process_image ball_chaser_generate_messages_cpp)





# From https://github.com/ros-planning/moveit2_tutorials/tree/main/doc/move_group_interface 
# include_directories(include ${catkin_INCLUDE_DIRS})
# 
# add_executable(move_group_interface_tutorial
#     src/move_group_interface_tutorial.cpp)
# target_include_directories(move_group_interface_tutorial
#     PUBLIC include)
# ament_target_dependencies(move_group_interface_tutorial
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# 
# install(TARGETS move_group_interface_tutorial
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )










# Pieces of CMakeLists.txt from the robowork folders
# cmake_minimum_required(VERSION 2.8.3)
# project(robowork_planning)
# 
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF) #-std=c++14 vs -std=gnu++14












# Combined Version
cmake_minimum_required(VERSION 2.8.3)
project(boat_planner)
# add_definitions(-std=c++11) 
# add_compile_options(-std=c++11)
# include_directories(include ${catkin_INCLUDE_DIRS})
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF) #-std=c++14 vs -std=gnu++14
find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  # message_generation
  # nodelet
  roscpp
  std_msgs
  # geometry_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES boat_planner
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)


# include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(boat_planner_node boat_planner_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(boat_planner_node ${catkin_LIBRARIES})

# add_dependencies(boat_planner_node boat_planner_generate_messages_cpp)

# install(DIRECTORY config launch scripts
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
