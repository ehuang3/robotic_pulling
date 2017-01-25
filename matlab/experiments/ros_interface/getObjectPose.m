function pose2d = getObjectPose2d(mocap_sub)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get Object pose in 2D from the Mocap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Constants();
  pose = receive(mocap_sub);
  object_pose = transformObject(pose.BodyPoses.Poses(1),Constants.OBJ_TFORM);
  % Throw away the z axis and keep the xy location of the object. 

  pose2d(1) = object_pose.Position.X;
  pose2d(2) = object_pose.Position.Y;
  pose2d(3) = 0;

end

