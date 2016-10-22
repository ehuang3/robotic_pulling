function [ t ] = ccm( a, b )
%CCM Find zeros of a second order trigonometric polynomial.
%   

%%
%f = @(t,a,b) a(1) + a(2)*cos(t) + a(3)*cos(2*t) + b(1)*sin(t) + b(2)*sin(2*t);
FFM = [ 0 1 0 0; 0 0 1 0; 0 0 0 1; ...
       -1*(a(3)+1i*b(2))/(a(3)-1i*b(2)), ...
       -1*(a(2)+1i*b(1))/(a(3)-1i*b(2)), ...
       -1*(2*a(1))/(a(3)-1i*b(2)), ...
       -1*(a(2)-1i*b(1))/(a(3)-1i*b(2))];
ev = eig(FFM);
t = angle(ev) - 1i*log(abs(ev));

end

