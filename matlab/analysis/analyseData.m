close all;
clear all;

import data.*

folder = fullfile(getDataPath,'experiment','small_rect8cp/')
% folder = '../../../data/mocap/';

%% Constants
w = 0.0496;
l = 0.0746;
%%
data = loadFiles(folder,w,l);
idx = findExclusion(data);
idx = idx(1:end-1)
% idx = []
% plotErrorRectangles(data,idx);
% errorDistribution(data,idx)

%%
playback(data,77);

% plotBounds(data,1)