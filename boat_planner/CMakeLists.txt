
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



# Combined Version
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(boat_planner_node boat_planner_node.cpp)






