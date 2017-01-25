function out = transformObject(ini,tform)

  t_in = [in.Position.X;in.Position.Y;in.Position.Z;1];

  R_in = quat2rotm([in.Orientation.W in.Orientation.X in.Orientation.Y in.Orientation.Z]);

  H_in = [[R t]; 0 0 0 1];

  H_out = tform*H_in;

  q_out = rotm2quat(H_out(1:3,1:3));


  %Copy In to Out
  out = rosmessage('geometry_msgs/Pose');
  out.Position.X = H_out(1,4);
  out.Position.Y = H_out(2,4);
  out.Position.Z = H_out(3,4);

  out.Orientation.W = q_out(0);
  out.Orientation.X = q_out(1);
  out.Orientation.Y = q_out(2);
  out.Orientation.z = q_out(3);

end

  


