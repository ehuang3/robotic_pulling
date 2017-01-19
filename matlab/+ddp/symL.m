function [ L, Lval ] = symL( x, u, dt )
%SYML 
%   

%% 

vi = sym('v','real');
L = (vi*dt)^2;
if ~isempty(u)
    Lval = double(subs(L,vi,u(1)));
else
    Lval = NaN;
end

end

