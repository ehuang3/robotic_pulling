function [ I, Ival ] = symIDFT( w, a, b, f )
%SYMIDFT 
%   

%% 

omega = sym('w','real');

n = length(a)-1;
N = (1:n);

I = a(1)/2 + sum(a(2:end).*cos(2*pi*f.*N.*omega) + b(2:end).*sin(2*pi*f.*N.*omega));
vpa(I,2);

Ival = double(subs(I,omega,w));

end

