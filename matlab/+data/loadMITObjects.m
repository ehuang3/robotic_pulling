function [ objects ] = loadMITObjects( mit_path )
%LOADMITOBJECTS
%   

%% 
if nargin == 0
    mit_path = '/home/eric/hd/Dropbox/Dropbox/mit dataset';
end

%% Load MIT Pushing Dataset Objects.
import presspull.*

Obj = containers.Map;

% display('Loading MIT Pushing Dataset Objects');

% Add rectangles.
a = 0.0450;
b = 0.0450;
p1 = [a; b];
p2 = [-a;b];
p3 = [-a;-b];
p4 = [a;-b];
o = struct;
o.V = [p1 p2 p3 p4 p1];
o.K = [(1:4)' (2:5)'];
o.com = [0;0];
o.name = 'rect1';
Obj('rect1') = o;

a = 0.0450;
b = 0.0563;
p1 = [a; b];
p2 = [-a;b];
p3 = [-a;-b];
p4 = [a;-b];
o = struct;
o.V = [p1 p2 p3 p4 p1];
o.K = [(1:4)' (2:5)'];
o.com = computeCoMPolygon(o.V);
o.name = 'rect2';
Obj('rect2') = o;

a = 0.0675;
b = 0.0450;
p1 = [a; b];
p2 = [-a;b];
p3 = [-a;-b];
p4 = [a;-b];
o = struct;
o.V = [p1 p2 p3 p4 p1];
o.K = [(1:4)' (2:5)'];
o.com = computeCoMPolygon(o.V);
o.name = 'rect3';
Obj('rect3') = o;

% Add ellipses.
t = linspace(0,2*pi);
a = 0.0525;
b = 0.0525;
o = struct;
o.V = [a*cos(t); b*sin(t)];
o.K = [(1:99)' (2:100)'];
o.com = computeCoMPolygon(o.V);
o.name = 'ellip1';
Obj('ellip1') = o;

a = 0.0525;
b = 0.0654;
o = struct;
o.V = [a*cos(t); b*sin(t)];
o.K = [(1:99)' (2:100)'];
o.com = computeCoMPolygon(o.V);
o.name = 'ellip2';
Obj('ellip2') = o;

a = 0.0525;
b = 0.0785;
o = struct;
o.V = [a*cos(t); b*sin(t)];
o.K = [(1:99)' (2:100)'];
o.com = computeCoMPolygon(o.V);
o.name = 'ellip3';
Obj('ellip3') = o;

% Add triangles.
A = [0.0450; 0.0450];
B = [-0.0809; 0.0450];
C = [0.0450; -0.0809];
o = struct;
o.V = [A B C A];
o.K = [(1:3)' (2:4)'];
o.com = computeCoMPolygon(o.V);
o.name = 'tri1';
Obj('tri1') = o;

A = [0.0450; 0.0450];
B = [-0.1060; 0.0450];
C = [0.0450; -0.1060];
o = struct;
o.V = [A B C A];
o.K = [(1:3)' (2:4)'];
o.com = computeCoMPolygon(o.V);
o.name = 'tri2';
Obj('tri2') = o;

A = [0.0450; 0.0450];
B = [-0.1315; 0.0450];
C = [0.0450; -0.0806];
o = struct;
o.V = [A B C A];
o.K = [(1:3)' (2:4)'];
o.com = computeCoMPolygon(o.V);
o.name = 'tri3';
Obj('tri3') = o;

% Add hex.
s = 0.0605;
p0 = s*[1; 0];
p1 = s*[cos(pi/3); sin(pi/3)];
p2 = s*[cos(2*pi/3); sin(2*pi/3)];
p3 = s*[cos(pi); sin(pi)];
p4 = s*[cos(4*pi/3); sin(4*pi/3)];
p5 = s*[cos(5*pi/3); sin(5*pi/3)];
o = struct;
o.V = [p0 p1 p2 p3 p4 p5 p0];
o.K = [(1:6)' (2:7)'];
o.com = computeCoMPolygon(o.V);
o.name = 'hex';
Obj('hex') = o;

% Add butter.
warning('WARNING: Missing BUTTER object');

% Convert map to struct.
keys = Obj.keys;
for i = 1:length(keys)
    objects(i) = Obj(keys{i});
end

end

