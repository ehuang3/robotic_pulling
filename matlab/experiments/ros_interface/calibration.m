%Calibrate Mocap:
% Put Object aligned with coordinate transform
% mocap = rossubscriber('/Mocap');

msg = receive(mocap);
x = 1e3*msg.BodyPoses.Poses(1).Position.X;
y = 1e3*msg.BodyPoses.Poses(1).Position.Y;
z = 1e3*msg.BodyPoses.Poses(1).Position.Z;
t = [x;y;z];
w = msg.BodyPoses.Poses(1).Orientation.W;
x = msg.BodyPoses.Poses(1).Orientation.X;
y = msg.BodyPoses.Poses(1).Orientation.Y;
z = msg.BodyPoses.Poses(1).Orientation.Z;
q = [x y z w];
R = quat2rotm(q);

H_in = [R, -R*t; 0 0 0 1]
H_out = [1 0 0 -760;
         0 1 0 0;
         0 0 1 -226;
         0 0 0 1;];
     
H_body_robot = [R t; 0 0 0 1];
H_robot_corner =  [ 1 0 0 -760;
                    0 1 0 0;
                    0 0 1 -226;
                    0 0 0 1;];
H_body_corner = H_robot_corner * H_body_robot;

p0 = [760; 0; 226; 1];


H = H_out*inv(H_in)
q = rotm2quat(H(1:3,1:3))
t = H(1:3,4)'
for i=1:100
         pause
         msg = receive(mocap);
         x = 1e3*msg.BodyPoses.Poses(1).Position.X;
         y = 1e3*msg.BodyPoses.Poses(1).Position.Y;
         z = 1e3*msg.BodyPoses.Poses(1).Position.Z;
         t = [x;y;z];
         w = msg.BodyPoses.Poses(1).Orientation.W;
         x = msg.BodyPoses.Poses(1).Orientation.X;
         y = msg.BodyPoses.Poses(1).Orientation.Y;
         z = msg.BodyPoses.Poses(1).Orientation.Z;
         q = [x y z w];
         R = quat2rotm(q);
         
         
         
         H_body_robot = [R t; 0 0 0 1];
         H_corner_robot = H_body_robot * inv(H_body_corner)

         H_in = [R, -R*t; 0 0 0 1];
         p0_in = H_in * p0;

         % H_out = H*H_in
         H*H_in*p0;
end
