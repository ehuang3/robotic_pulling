function [ Lf, Lfx, Lfxx, Lfval ] = symLf( xn, xf, del, kel )
%SYMLF 
%   

%% 

x = sym('x','real');
y = sym('y','real');
u = sym('u','real');
l = sym('l','real');

d = sym('d%d',[1 4],'real');
k = sym('kf%d',[1 4],'real');

fx = sym('xf','real');
fy = sym('yf','real');
fu = sym('uf','real');
fl = sym('lf','real');

% Lf = sum(d.^2.*(sqrt(1+(([x y u l] - [fx fy fu fl])./d).^2)-1));
Lf = sum(k.*(sqrt([x-fx,y-fy,u-fu,l-fl].^2+d.^2)-d));
Lfx = gradient(Lf, [x y u l]);
Lfxx = hessian(Lf, [x y u l]);
if ~isempty(xn) && ~isempty(xf)
    Lfval = double(subs(Lf,[x y u l xf yf uf lf d],[xn' xf' del]));
else
    Lfval = NaN;
end

end
