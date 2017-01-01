%% Check library dependencies.
clc
clear

import utils.*

src_path = '/home/eric/src/presspull/matlab';
M = getAllFiles(src_path);

for i = 1:length(M)
    m = M{i};
    warning(m)
    [fList,pList] = matlab.codetools.requiredFilesAndProducts(m);
    fList'
end