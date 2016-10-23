function [ t ] = anglePointRay( A, B, r_B )
%ANGLEPOINTRAY 
%   

%%
r_B = r_B / norm(r_B);
perp_rB = [-r_B(2); r_B(1)];
BA = A - B;
x = sum(BA.*r_B);
y = sum(BA.*perp_rB);
t = atan2(y,x);

end
