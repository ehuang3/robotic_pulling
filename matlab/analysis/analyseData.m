close all;
clear all;


<<<<<<< HEAD
folder = fullfile(getDataPath,'experiment','small_rect8cp/')
% folder = '../../../data/mocap/';
=======

% folder = fullfile(getDataPath,'experiment','tmp/')
folder = '../../../data/mocap/';
>>>>>>> be6e2890df1d53cb83b929627b980cd21b51da4e

%% Constants
w = 0.0496;
l = 0.0746;
%%
data = loadFiles(folder,w,l);
idx = findExclusion(data);
<<<<<<< HEAD
idx = idx(1:end-1)
=======
>>>>>>> be6e2890df1d53cb83b929627b980cd21b51da4e
% idx = []
% plotErrorRectangles(data,idx);
% errorDistribution(data,idx)

%%
playback(data,77);

% plotBounds(data,1)
