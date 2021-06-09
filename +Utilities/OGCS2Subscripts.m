function [col, row] = OGCS2Subscripts(ogcs_x, ogcs_y, resolution)
%OGCS2SUBSCRIPTS convert continuous CS to discrete grid subscripts
col = floor(ogcs_x/resolution) + 1; % +1 - Matlab indexing
row = floor(ogcs_y/resolution) + 1; % +1 - Matlab indexing
end

