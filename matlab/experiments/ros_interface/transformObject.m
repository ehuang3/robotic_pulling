function out = transformObject(in,tform)

  t_in = 1000*[in.Position.X;in.Position.Y;in.Position.Z];

  R_in = quat2rotm([in.Orientation.X in.Orientation.Y in.Orientation.Z in.Orientation.W]);

  H_in = [[R_in t_in]; 0 0 0 1];

  out = H_in*tform;
  %Copy In to Out

end

  


