function [ object ] = rectangle( a, b )
%RECTANGLE 
%   

%% 
import presspull.*
p1 = [a; b];
p2 = [-a;b];
p3 = [-a;-b];
p4 = [a;-b];
object.V = [p1 p2 p3 p4 p1];
object.K = [(1:4)' (2:5)'];
object.com = [0;0];

end

