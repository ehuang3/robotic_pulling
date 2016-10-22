function [ ] = plotRectangle( rect )
%PLOTRECTANGLE 
%   

%% Plot the rectangle.
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
tri = [(c(:,1) + 2 * c(:,2))/3, ... 
       (2 * c(:,1) + c(:,2))/3, ...
       (c(:,1) + c(:,2))/3, ...
       (c(:,1) + 2 * c(:,2))/3];
       
c(1,:) = c(1,:) + x;
c(2,:) = c(2,:) + y;
tri(1,:) = tri(1,:) + x;
tri(2,:) = tri(2,:) + y;

% Plot rectangle.
hold on
plot(c(1,:), c(2,:), 'b');
plot(tri(1,:), tri(2,:), 'b');
plot(x,y,'b+')

end

