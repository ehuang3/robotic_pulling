%% Plot angular velocity bounds.

clc
clear

import presspull.*
import data.*
import utils.*

%% File list of bounds.
data_path = getDataPath;
bounds_path = fullfile(data_path, 'bounds');
bounds_files = sort(getAllFiles(bounds_path));

%% Load a bounds.
i = 1+2*16
pfile = bounds_files{195 + i + 1}
load(pfile)
objp = obj;
n_V = size(objp.V,2);
objp.K = [(1:n_V-1)' (2:n_V)'];

bfile = bounds_files{195 + i}
load(bfile)

% Plot exact bounds.
subplot(211)
cla
hold on
axis equal
grid on
plot(obj.V(1,:),obj.V(2,:),'b')
plot(0,0,'ro')
plot(obj.com(1),obj.com(2),'kx')
plot(objp.V(1,:),objp.V(2,:),'k')
subplot(212)
cla
hold on
axis auto
grid on
plot(obj.t,obj.U,'b')
plot(obj.t,obj.L,'b')
plot(objp.t,objp.U,'k')
plot(objp.t,objp.L,'k')
