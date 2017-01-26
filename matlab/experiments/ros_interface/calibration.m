%Calibrate Mocap:
% Put Object aligned with coordinate transform
% mocap = rossubscriber('/Mocap');

msg = receive(mocap);
x = 1e3*msg.BodyPoses.Poses(1).Position.X;
y = 1e3*msg.BodyPoses.Poses(1).Position.Y;
z = 1e3*msg.BodyPoses.Poses(1).Position.Z;
t = [x;y;z;1];
w = msg.BodyPoses.Poses(1).Orientation.W;
x = msg.BodyPoses.Poses(1).Orientation.X;
y = msg.BodyPoses.Poses(1).Orientation.Y;
z = msg.BodyPoses.Poses(1).Orientation.Z;
q = [w x y z];
R = quat2rotm(q);

H_in = [[R;0 0 0] t]
H_out = [1 0 0 760;
         0 1 0 0;
         0 0 1 226;
         0 0 0 1;];


H = inv(H_in)*H_out
q = rotm2quat(H(1:3,1:3))
t = H(1:3,4)'
for i=1:10
         pause
         msg = receive(mocap);
         x = 1e3*msg.BodyPoses.Poses(1).Position.X;
         y = 1e3*msg.BodyPoses.Poses(1).Position.Y;
         z = 1e3*msg.BodyPoses.Poses(1).Position.Z;
         t = [x;y;z;1];
         w = msg.BodyPoses.Poses(1).Orientation.W;
         x = msg.BodyPoses.Poses(1).Orientation.X;
         y = msg.BodyPoses.Poses(1).Orientation.Y;
         z = msg.BodyPoses.Poses(1).Orientation.Z;
         q = [w x y z];
         R = quat2rotm(q);

         H_in = [[R;0 0 0] t]

         H_out = H_in*H
end
