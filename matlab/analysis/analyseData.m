close all;
clear all;



% folder = fullfile(getDataPath,'experiment','tmp/')
folder = '../../../data/mocap/';

%% Constants
w = 0.0496;
l = 0.0746;
%%
data = loadFiles(folder,w,l);
idx = findExclusion(data);
% idx = []
% plotErrorRectangles(data,idx);
% errorDistribution(data,idx)

playback(data,1);
