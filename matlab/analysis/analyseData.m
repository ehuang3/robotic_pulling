close all;
clear all;

folder = '../../../data/4cp/';

%% Constants

w = 0.048;
l = 0.073;
%%
data = loadFiles(folder,w,l);
idx = findExclusion(data);
plotErrorRectangles(data,idx);
errorDistribution(data,idx)
