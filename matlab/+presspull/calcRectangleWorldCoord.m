function [ pt ] = calcRectangleWorldCoord( pt, rect )
%CALCRECTANGLEWORLDCOORD 

%%
x = rect(1);
y = rect(2);
theta = rect(3);
% l = rect(4);
% w = rect(5);

rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
pt = rot * pt + [x;y];


end

