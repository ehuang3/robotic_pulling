function out = sendAndExecuteTrajectory(trajectory,robot,safe_z,interact_z)

  % Go to the first trajectory point at SAFE_Z

  request = rosmessage(robot.setcartesian_client);
  request.X = trajectory(1,1);
  request.Y = trajectory(1,2);
  request.Z = safe_z;
  request.Q0 = 0;
  request.Qx = 0;
  request.Qy = -1;
  request.Qz = 0;

  response = call(robot.setcartesian_client,request);

  %Add Trajectory to robot:
  

  
  speed_req = rosmessage(robot.setspeed_client);
  speed_req.Tcp = 20;
  response = call(robot.setspeed_client,speed_req);
  add_request = rosmessage(robot.setcartesian_client);

  add_request.X = trajectory(1,1);
  add_request.Y = trajectory(1,2);
  add_request.Z = interact_z;
  add_request.Q0 = 0;
  add_request.Qx = 0;
  add_request.Qy = -1;
  add_request.Qz = 0;

  response = call(robot.setcartesian_client,add_request);
  speed_req = rosmessage(robot.setspeed_client);
  speed_req.Tcp = 50;
  global mocap_data;
  mocap_data = [];
  rossubscriber('/Mocap',@getMocapData);
  for i=2:size(trajectory,1)

    add_request = rosmessage(robot.setcartesian_client);

    add_request.X = trajectory(i,1);
    add_request.Y = trajectory(i,2);
    add_request.Z = interact_z;
    add_request.Q0 = 0;
    add_request.Qx = 0;
    add_request.Qy = -1;
    add_request.Qz = 0;

    response = call(robot.setcartesian_client,add_request);
  end
  sub = [];
  
  request = rosmessage(robot.setcartesian_client);
  request.X = trajectory(end,1);
  request.Y = trajectory(end,2);
  request.Z = safe_z;
  request.Q0 = 0;
  request.Qx = 0;
  request.Qy = -1;
  request.Qz = 0;

  response = call(robot.setcartesian_client,request);
  
  
  request = rosmessage(robot.setcartesian_client);
  request.X = 450;
  request.Y = 0;
  request.Z = 647;
  request.Q0 = 0;
  request.Qx = 0;
  request.Qy = -1;
  request.Qz = 0;

  response = call(robot.setcartesian_client,request);
  

end

function getMocapData(src,msg)
  global mocap_data;
  mocap_data = [mocap_data msg];
end

    
