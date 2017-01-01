%% Test LoadMITObjects.
import data.*

O = loadMITObjects;

for i = O.keys()
    i{:}
    O(i{:}).com
end