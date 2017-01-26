function out = transformObject(ini,tform)

  t_in = [in.Position.X;in.Position.Y;in.Position.Z;1];

  R_in = quat2rotm([in.Orientation.W in.Orientation.X in.Orientation.Y in.Orientation.Z]);

  H_in = [[R t]; 0 0 0 1];

  out = H_in*tform;
  %Copy In to Out

end

  


