cmake_minimum_required(VERSION 2.8.3)


project(robot_comm)

find_package(catkin REQUIRED 
  COMPONENTS roscpp matvec geometry_msgs message_generation
  )

add_message_files(FILES
  robot_CartesianLog.msg
  robot_ForceLog.msg
  robot_JointsLog.msg
  )
add_service_files(
  FILES
  robot_Approach.srv
  robot_GetCartesian.srv
  robot_GetFK.srv
  robot_GetIK.srv
  robot_GetJoints.srv
  robot_GetState.srv
  robot_IsMoving.srv
  robot_Ping.srv
  robot_SetCartesian.srv
  robot_SetCartesianJ.srv
  robot_SetComm.srv
  robot_SetDefaults.srv
  robot_SetJoints.srv
  robot_SetSpeed.srv
  robot_SetTool.srv
  robot_SetTrackDist.srv
  robot_SetVacuum.srv
  robot_SetWorkObject.srv
  robot_SetZone.srv
  robot_SpecialCommand.srv
  robot_Stop.srv
  robot_AddTrajectoryPoint.srv
  robot_ExecuteTrajectory.srv
  robot_ClearTrajectory.srv
  )
generate_messages(
  DEPENDENCIES
  geometry_msgs
  )

include_directories(include/robot_comm)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${matvec_INCLUDE_DIRS}
  ${robot_comm_INCLUDE_DIRS}
  )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_comm
  CATKIN_DEPENDS roscpp matvec geometry_msgs message_runtime
  )


add_library(robot_comm src/robot_comm.cpp)
add_dependencies(robot_comm ${PROJECT_NAME}_gencpp)
target_link_libraries(robot_comm matvec)
target_link_libraries(robot_comm ${catkin_LIBRARIES})
install (DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
