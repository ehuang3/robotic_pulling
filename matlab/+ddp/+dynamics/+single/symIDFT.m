function [ I ] = symIDFT( w, a, b, f )
%SYMIDFT 
%   

%% 

n = length(a)-1;
N = (1:n);
I = a(1)/2 + sum(a(2:end).*cos(2*pi*f.*N.*w) + b(2:end).*sin(2*pi*f.*N.*w));

end

