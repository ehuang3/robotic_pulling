function [ L, Lval ] = symL( x, u, dt )
%SYML 
%   

%% 

vi = sym('v','real');
% L = (vi*dt)^2;
L = 0*vi;
if ~isempty(u)
    Lval = double(subs(L,vi,u(1)));
else
    Lval = NaN;
end

end

