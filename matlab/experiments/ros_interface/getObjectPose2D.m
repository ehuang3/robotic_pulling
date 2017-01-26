function pose2d = getObjectPose2D(mocap_sub)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get Object pose in 2D from the Mocap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Constants();
  pose = receive(mocap_sub);
  object_pose = transformObject(pose.BodyPoses.Poses(1),Constants.OBJ_TFORM);
  % Throw away the z axis and keep the xy location of the object. 

  pose2d(1) = object_pose(1,4);
  pose2d(2) = object_pose(2,4);
  pose2d(3) = acos(object_pose(1));

end

