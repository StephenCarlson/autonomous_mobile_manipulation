cmake_minimum_required(VERSION 2.8.3)
project(boat_planner)

add_definitions(-std=c++11) 
add_compile_options(-std=c++11)
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



include_directories(include ${catkin_INCLUDE_DIRS})


# # Library
# add_library(boat_planner_lib src/boat_planner_node.cpp)
# target_link_libraries(boat_planner_lib ${catkin_LIBRARIES})
# 
# # Standalone node
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${PROJECT_NAME}_lib)
# roslaunch_add_file_check(launch)





add_executable(boat_planner_node src/boat_planner_node.cpp)
target_link_libraries(boat_planner_node ${catkin_LIBRARIES})
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