function [ value, isterminal, direction ] = convergeEvent( t, y, target, dir )
%CONVERGEEVENT 
%   

%% 
value = y - target;
isterminal = 1;
direction = dir;

end

