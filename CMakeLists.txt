cmake_minimum_required(VERSION 2.8.3)
project(role_playing_robots)


################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   rospy 
   std_msgs 
   geometry_msgs
   message_generation
   anki_vector_ros
)

## Generate messages in the 'msg' folder
# add_message_files (
#    FILES
# )

generate_messages (
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package()
catkin_package(CATKIN_DEPENDS message_runtime)


## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   Line.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
 #  FILES
#   Action1.action
#   Action2.action
# )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############



#############
## Testing ##
#############
