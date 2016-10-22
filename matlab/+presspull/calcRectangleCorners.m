function [ c ] = calcRectangleCorners( rect )
%CALCRECTANGLECORNERS 
%   

%%
x = rect(1);
y = rect(2);
theta = rect(3);
l = rect(4);
w = rect(5);

% Get the four corner points.
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
c = [l/2 l/2 -l/2 -l/2; w/2 -w/2 -w/2 w/2];
c = [c c(:,1)];
c = rot * c;
c(1,:) = c(1,:) + x;
c(2,:) = c(2,:) + y;

end

