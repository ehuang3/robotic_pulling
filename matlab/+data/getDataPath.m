function [ path ] = getDataPath( )
%GETDATAPATH 
%   

%% 
path = mfilename('fullpath');
[path, ~, ~] = fileparts(path);
[path, ~, ~] = fileparts(path);
[path, ~, ~] = fileparts(path);
path = fullfile(path, 'data');

end

