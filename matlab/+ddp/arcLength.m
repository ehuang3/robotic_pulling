function [ l, t ] = arcLength( v1, v2, d )
%ARCLENGTH 
%   d - 1 = left, 0 = right

t = atan2(v2(2),v2(1)) - atan2(v1(2),v1(1));
if t < 0 && d
    t = t + 2*pi;
elseif t > 0 && ~d
    t = t - 2*pi;
end
l = abs(t * norm(v1));

end

