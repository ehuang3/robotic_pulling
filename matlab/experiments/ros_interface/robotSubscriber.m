function robot = robotSubscriber()
  %Robot Service Clients
  robot.addtrajpt_client = rossvcclient('/robot_AddTrajectoryPoint'); 
  robot.cleartraj_client = rossvcclient('/robot_ClearTrajectory');
  robot.exectraj_client = rossvcclient('/robot_ExecuteTrajectory');
  robot.getcartesian_client = rossvcclient('/robot_GetCartesian');
  robot.setcartesian_client = rossvcclient('/robot_SetCartesian');
  robot.settool_client = rossvcclient('/robot_SetTool');
  robot.setworkobj_client = rossvcclient('/robot_SetWorkObject');
  robot.status_client = rossvcclient('/robot_GetState');
  robot.splcmd_client = rossvcclient('/robot_SpecialCommand');
end
