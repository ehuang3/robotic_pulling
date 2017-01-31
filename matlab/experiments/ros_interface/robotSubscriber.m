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
  robot.setspeed_client = rossvcclient('/robot_SetSpeed');
  setToolAndWorkObj(robot);
end
function []= setToolAndWorkObj(robot)
    %Set tool and workobject to unity
    request = rosmessage(robot.setworkobj_client)
    request.X = 0;
    request.Y = 0;
    request.Z = 0;
    request.Q0 = 1;
    request.Qx = 0;
    request.Qy = 0;
    request.Qz = 0;

    response = call(robot.setworkobj_client,request);
    
    request = rosmessage(robot.settool_client);
    request.X = 0;
    request.Y = 0;
    request.Z = 0;
    request.Q0 = 1;
    request.Qx = 0;
    request.Qy = 0;
    request.Qz = 0;

    response = call(robot.settool_client,request);
end
