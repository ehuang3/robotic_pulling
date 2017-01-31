function data = mocapToPose(mocap_data)
  data = zeros(3,numel(mocap_data));
  for i=1:numel(mocap_data)
    object_pose = transformObject(pose.BodyPoses.Poses(1),object_transform);
    % Throw away the z axis and keep the xy location of the object. 

    pose2d(1) = object_pose(1,4) / 1000;
    pose2d(2) = object_pose(2,4) / 1000;
    pose2d(3) = atan2(object_pose(2,1),object_pose(1));
    data(:,i) = pose2d
  end
end

