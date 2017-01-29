close all;
clear all;

folder = '../../../data/';

%% Constants

w = 0.05113;
l = 0.0762;

data = loadFiles(folder,w,l);
% plotErrorRectangles(data);
% figure()
% for i = 51:numel(data.dat)
%   plotTrajectory(data,i);
%   pause
%   cla
% end


% err = sqrt(sum((e).^2,2)/numel(files));
i = 38;
playback(data,i);
