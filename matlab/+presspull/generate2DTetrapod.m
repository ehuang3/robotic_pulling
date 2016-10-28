function [ X, Y, K ] = generate2DTetrapod( h, l, cut )
%GENERATE2DTETRAPOD 
%   

%% 
import presspull.*

if nargin == 0
    h = 0.5;
    l = 0.7;
    cut = 0;
end
if nargin == 2
    cut = 0;
end

R = [0 l l 0; h/2 h/2 -h/2 -h/2];

t = pi/3;
rot = [cos(t) -sin(t); sin(t) cos(t)];
R_1 = rot * R;
% plot(R_1(1,:),R_1(2,:))

t = pi;
rot = [cos(t) -sin(t); sin(t) cos(t)];
R_2 = rot * R;
% plot(R_2(1,:),R_2(2,:))

t = -pi/3;
rot = [cos(t) -sin(t); sin(t) cos(t)];
R_3 = rot * R;
% plot(R_3(1,:),R_3(2,:))

P = R_1;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_2;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_1 = A + t * AB;

P = R_2;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_3;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_2 = A + t * AB;

P = R_3;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_1;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_3 = A + t * AB;

XY = [X_1 R_2(:,3) R_2(:,2) X_2 R_3(:,3) R_3(:,2) ... 
      X_3 R_1(:,3) R_1(:,2) X_1]';
X = XY(:,1);
Y = XY(:,2);
K = [(1:9)' (2:10)'];
% plot(X,Y,'r')

% Cut a hexagon from center.
P = R_1;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_3;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_4 = A + t * AB;

P = R_2;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_1;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_5 = A + t * AB;

P = R_3;
A = [P(1,1); P(2,1)];
B = [P(1,2); P(2,2)];
Q = R_2;
C = [Q(1,4); Q(2,4)];
D = [Q(1,3); Q(2,3)];
AB = B - A;
CD = D - C;
[t,~] = intersectLineLine(A,AB,C,CD);
X_6 = A + t * AB;

t = pi/6;
rot = [cos(t) -sin(t); sin(t) cos(t)];
XY2 = 0.5 .* rot *[X_1 X_4 X_2 X_5 X_3 X_6 X_1];
X2 = XY2(1,:)';
Y2 = XY2(2,:)';
K2 = 10 + [(1:6)' (2:7)'];
% plot(X2,Y2,'r')

if cut 
    X = [X; X2];
    Y = [Y; Y2];
    K = [K; K2];
end

end

