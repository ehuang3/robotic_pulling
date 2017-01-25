function out = sendAndExecuteTrajectory(trajectory,robot)
  Constants();

  % Go to the first trajectory point at SAFE_Z

  request = rosmessage(robot.setcartesian_client);
  request.x = trajectory(1,1);
  request.y = trajectory(1,2);
  request.z = SAFE_Z;
  request.q0 = 0;
  request.qx = 1;
  request.qy = 0;
  request.qz = 0;

  response = call(robot.setcartesian_client,request);

  %Add Trajectory to robot:

  for i=1:size(trajectory,1)

    add_request = rosmessage(robot.addtrajpt_client);

    add_request.x = trajectory(i,1) + HOLE_LOCATION(1);
    add_request.y = trajectory(i,2) + HOLE_LOCATION(2);
    add_request.z = SAFE_Z;
    add_request.q0 = 0;
    add_request.qx = 1;
    add_request.qy = 0;
    add_request.qz = 0;
    add_request.time = 0.1;
    add_request.zone = 1;

    response = call(robot.setcartesian_client,add_request);
  end


  %% Execute Trajectory:

  exec_request = rosmessage(robot.exectraj_client);

  exec_response = call(robot_exectraj_client,exec_request);

  out = exec_response
end
