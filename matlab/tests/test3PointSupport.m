%% Rotation center of three point support.

s = linspace(0, 2*pi);
t = log(abs(sec(s) + tan(s)))
plot(t,s)

x = linspace(-1,1);
y = linspace(-1,1);
xr = 0.5;
yr = 0;
[X,Y] = meshgrid(x,y);
F = (X.*X + Y.*Y - X.*xr) ./ sqrt((X - xr).^2  + Y.*Y);
surf(X,Y,F)
xlabel('x')
ylabel('y')

% F = (2.*X-xr).*((X-xr).^2+Y.^2)-(X-xr).*(X.^2+Y.^2-X.*xr);
% F = F./sqrt((X - xr).^2  + Y.*Y).^3;
% surf(X,Y,F)
% 
% F = (2.*Y).*((X-xr).^2+Y.^2)-Y.*(X.^2+Y.^2-X.*xr);
% F = F./sqrt((X - xr).^2  + Y.*Y).^3;
% surf(X,Y,F)
% xlabel('x')
% ylabel('y')
% 
% F = X.*X - 3.*X.*xr + 2.*xr.^2 + Y.*Y;
% surf(X,Y,F)
% xlabel('x')
% ylabel('y')

%%
x = -2:0.25:2;
y = x;
[X,Y] = meshgrid(x);

F = X.*exp(-X.^2-Y.^2);
surf(X,Y,F)
